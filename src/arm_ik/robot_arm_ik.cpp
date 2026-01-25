#ifndef PINOCCHIO_WITH_CASADI
    #define PINOCCHIO_WITH_CASADI
#endif

#include <pinocchio/autodiff/casadi.hpp>
#include "arm_ik/robot_arm_ik.h"

#include <stdexcept>
#include <cmath>

// ============================================================================
// G1_29_ArmIK Implementation
// ============================================================================

namespace IK {
G1_29_ArmIK::G1_29_ArmIK(bool unit_test, bool visualization, 
                         const RobotConfig* robot_config)
    : unit_test_(unit_test), 
      visualization_(visualization) 
      {
    
    if (robot_config == nullptr) {
        robot_config_ = {
            "../assets/g1/g1_29dof_with_hand_rev_1_0.urdf",
            "../assets/g1/"
        };
    } else {
        robot_config_ = *robot_config;
    }

    urdf_path_ = robot_config_.asset_file;
    model_dir_ = robot_config_.asset_root;
    
    std::cout << std::fixed << std::setprecision(5);
        
    // Initialize joints to lock
    initialize_joints_to_lock();
    
    std::cout << "[G1_29_ArmIK] >>> Loading URDF from " << urdf_path_ << std::endl;

    // Build robot model from URDF
    pinocchio::urdf::buildModel(urdf_path_, robot_model_);
    robot_data_ = pinocchio::Data(robot_model_);
    
    // Build reduced robot by locking specified joints
    std::vector<pinocchio::JointIndex> joints_to_lock;
    for (const auto& joint_name : mixed_joints_to_lock_ids_) {
        if (robot_model_.existJointName(joint_name)) {
            joints_to_lock.push_back(robot_model_.getJointId(joint_name));
        }
    }
    
    Eigen::VectorXd reference_config = Eigen::VectorXd::Zero(robot_model_.nq);
    pinocchio::buildReducedModel(robot_model_, joints_to_lock, 
                                    reference_config, reduced_model_);
    reduced_data_ = pinocchio::Data(reduced_model_);
    
    // Add end-effector frames
    add_end_effector_frames();

    // Extracting camera frames
    oMcamera = reduced_model_.getFrameId("d435_link", pinocchio::BODY);
    oMLidar = reduced_model_.getFrameId("mid360_link", pinocchio::BODY);
    
    // Setup CasADi optimization
    setup_optimization();
    
    // Initialize filter and data
    Eigen::VectorXd weights(4);
    weights << 0.4, 0.3, 0.2, 0.1;
    smooth_filter_ = std::make_unique<WeightedMovingFilter>(weights, 14);
    init_data_ = Eigen::VectorXd::Zero(reduced_model_.nq);
}

G1_29_ArmIK::~G1_29_ArmIK() {}

void G1_29_ArmIK::initialize_joints_to_lock() {
    mixed_joints_to_lock_ids_ = {
        "left_hip_pitch_joint",
        "left_hip_roll_joint", 
        "left_hip_yaw_joint",
        "left_knee_joint", 
        "left_ankle_pitch_joint", 
        "left_ankle_roll_joint",
        "right_hip_pitch_joint", 
        "right_hip_roll_joint", 
        "right_hip_yaw_joint",
        "right_knee_joint", 
        "right_ankle_pitch_joint", 
        "right_ankle_roll_joint",
        "waist_yaw_joint", 
        "waist_roll_joint", 
        "waist_pitch_joint",

        "left_hand_thumb_0_joint", 
        "left_hand_thumb_1_joint", 
        "left_hand_thumb_2_joint",
        "left_hand_middle_0_joint", 
        "left_hand_middle_1_joint",
        "left_hand_index_0_joint", 
        "left_hand_index_1_joint",
        "right_hand_thumb_0_joint", 
        "right_hand_thumb_1_joint", 
        "right_hand_thumb_2_joint",
        "right_hand_index_0_joint", 
        "right_hand_index_1_joint",
        "right_hand_middle_0_joint", 
        "right_hand_middle_1_joint"
    };
}

void G1_29_ArmIK::add_end_effector_frames() {
    // Add left end-effector frame
    left_wrist_id_ = reduced_model_.getJointId("left_wrist_yaw_joint");
    pinocchio::SE3 left_placement(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.05, 0, 0));
    reduced_model_.addFrame(pinocchio::Frame("L_ee", left_wrist_id_, left_placement, 
                                            pinocchio::OP_FRAME));
    
    // Add right end-effector frame
    right_wrist_id_ = reduced_model_.getJointId("right_wrist_yaw_joint");
    pinocchio::SE3 right_placement(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.05, 0, 0));
    reduced_model_.addFrame(pinocchio::Frame("R_ee", right_wrist_id_, right_placement, 
                                             pinocchio::OP_FRAME));

    // Get frame IDs for end effectors
    L_hand_id_ = reduced_model_.getFrameId("L_ee");
    R_hand_id_ = reduced_model_.getFrameId("R_ee");
}

void G1_29_ArmIK::setup_optimization() {
    #ifdef USE_CASADI
    // Create optimization variables and parameters
    var_q_ = opti_.variable(reduced_model_.nq, 1);
    var_q_last_ = opti_.parameter(reduced_model_.nq, 1);
    param_tf_l_ = opti_.parameter(4, 4);
    param_tf_r_ = opti_.parameter(4, 4);
    
    // Set joint limits as constraints
    casadi::DM lower_limits = eigen_to_casadi(reduced_model_.lowerPositionLimit);
    casadi::DM upper_limits = eigen_to_casadi(reduced_model_.upperPositionLimit);
    opti_.subject_to(opti_.bounded(lower_limits, var_q_, upper_limits));

    pinocchio::ModelTpl<casadi::SX> cmodel(reduced_model_.cast<casadi::SX>());
    pinocchio::DataTpl<casadi::SX> cdata(cmodel);

    auto L_hand_id = cmodel.getFrameId("L_ee");
    auto R_hand_id = cmodel.getFrameId("R_ee");

    casadi::SX cq = casadi::SX::sym("q", cmodel.nq);
    casadi::SX cTf_l = casadi::SX::sym("tf_l", 4, 4);
    casadi::SX cTf_r = casadi::SX::sym("tf_r", 4, 4);

    Eigen::Matrix<casadi::SX, Eigen::Dynamic, 1> cq_eigen(cmodel.nq);
    for (int i = 0; i < cmodel.nq; ++i) {
        cq_eigen(i) = cq(i);
    }

    pinocchio::framesForwardKinematics(cmodel, cdata, cq_eigen);
    pinocchio::SE3Tpl<casadi::SX> T_L_se3 = cdata.oMf[L_hand_id];
    pinocchio::SE3Tpl<casadi::SX> T_R_se3 = cdata.oMf[R_hand_id];

    casadi::SX rot_L(casadi::Sparsity::dense(3,3));
    casadi::SX rot_R(casadi::Sparsity::dense(3,3));
    casadi::SX trans_L(casadi::Sparsity::dense(3,1));
    casadi::SX trans_R(casadi::Sparsity::dense(3,1));
    casadi::SX target_rot_L(casadi::Sparsity::dense(3,3));
    casadi::SX target_rot_R(casadi::Sparsity::dense(3,3));
    casadi::SX target_trans_L(casadi::Sparsity::dense(3,1));
    casadi::SX target_trans_R(casadi::Sparsity::dense(3,1));
    Eigen::Matrix<casadi::SX, 3, 3> rot_err_mat_L;
    Eigen::Matrix<casadi::SX, 3, 3> rot_err_mat_R;
    casadi::SX rot_err_L(casadi::Sparsity::dense(3,1));
    casadi::SX rot_err_R(casadi::Sparsity::dense(3,1));
    casadi::SX trans_err_L(casadi::Sparsity::dense(3,1));
    casadi::SX trans_err_R(casadi::Sparsity::dense(3,1));

    pinocchio::casadi::copy(T_L_se3.rotation(), rot_L);
    pinocchio::casadi::copy(T_R_se3.rotation(), rot_R);
    pinocchio::casadi::copy(T_L_se3.translation(), trans_L);
    pinocchio::casadi::copy(T_R_se3.translation(), trans_R);

    target_rot_L = cTf_l(casadi::Slice(0,3), casadi::Slice(0,3));;
    target_rot_R = cTf_r(casadi::Slice(0,3), casadi::Slice(0,3));;
    target_trans_L = cTf_l(casadi::Slice(0,3), 3);
    target_trans_R = cTf_r(casadi::Slice(0,3), 3);

    pinocchio::casadi::copy(casadi::SX::mtimes(rot_L, target_rot_L.T()), rot_err_mat_L);
    pinocchio::casadi::copy(casadi::SX::mtimes(rot_R, target_rot_R.T()), rot_err_mat_R);

    pinocchio::casadi::copy(pinocchio::log3(rot_err_mat_L), rot_err_L);
    pinocchio::casadi::copy(pinocchio::log3(rot_err_mat_R), rot_err_R);
    trans_err_L = trans_L - target_trans_L;
    trans_err_R = trans_R - target_trans_R;

    // Translation Error
    casadi::SX trans_err = casadi::SX::vertcat({
        trans_err_L,
        trans_err_R
    });
    casadi::Function translational_error = casadi::Function("trans_err", {cq, cTf_l, cTf_r}, {trans_err});

    // Rotational Error (using log3 for SO3 distance)
    casadi::SX rot_err = casadi::SX::vertcat({
        rot_err_L,
        rot_err_R
    });
    casadi::Function rotational_error = casadi::Function("rot_err", {cq, cTf_l, cTf_r}, {rot_err});

    // Costs
    casadi::MX cost_trans = casadi::MX::sumsqr(translational_error({var_q_, param_tf_l_, param_tf_r_})[0]);
    casadi::MX cost_rot = casadi::MX::sumsqr(rotational_error({var_q_, param_tf_l_, param_tf_r_})[0]);
    casadi::MX cost_reg = casadi::MX::sumsqr(var_q_);
    casadi::MX cost_smooth = casadi::MX::sumsqr(var_q_ - var_q_last_);

    opti_.minimize(50.0 * cost_trans + cost_rot + 0.02 * cost_reg + 0.1 * cost_smooth);
    
    // Set optimization options
    casadi::Dict opts;
    opts["expand"] = true;
    opts["detect_simple_bounds"] = true;
    opts["calc_lam_p"] = false;
    opts["print_time"] = false;
    opts["ipopt.sb"] = "yes";
    opts["ipopt.print_level"] = 0;
    opts["ipopt.max_iter"] = 50;
    opts["ipopt.tol"] = 1e-6;
    opts["ipopt.acceptable_tol"] = 5e-4;
    opts["ipopt.acceptable_iter"] = 5;
    opts["ipopt.warm_start_init_point"] = "yes";
    opts["ipopt.derivative_test"] = "none";
    opts["ipopt.jacobian_approximation"] = "exact";
    
    opti_.solver("ipopt", opts);
    #endif // USE_CASADI
}

std::pair<Eigen::Matrix4d, Eigen::Matrix4d> G1_29_ArmIK::scale_arms(
    const Eigen::Matrix4d& human_left_pose,
    const Eigen::Matrix4d& human_right_pose,
    double human_arm_length,
    double robot_arm_length) {
    
    double scale_factor = robot_arm_length / human_arm_length;
    Eigen::Matrix4d robot_left_pose = human_left_pose;
    Eigen::Matrix4d robot_right_pose = human_right_pose;
    
    robot_left_pose.block<3, 1>(0, 3) *= scale_factor;
    robot_right_pose.block<3, 1>(0, 3) *= scale_factor;
    
    return {robot_left_pose, robot_right_pose};
}

JointState G1_29_ArmIK::solve_ik(
    const Eigen::Matrix4d& left_wrist,
    const Eigen::Matrix4d& right_wrist,
    const Eigen::VectorXd* current_lr_arm_motor_q,
    const Eigen::VectorXd* current_lr_arm_motor_dq) {

    
    for (int j = 0; j < reduced_model_.njoints; ++j) {
        std::cout << "[G1_29_ArmIK] >>> Joint " << j << ": " 
                  << reduced_model_.names[j] << ": " 
                  << reduced_model_.joints[j].shortname() << std::endl;
    }

    std::cout << "[G1_29_ArmIK] >>> Model has nq=" << reduced_model_.nq << "and nv=" << reduced_model_.nv << std::endl;
    
    // Update initial guess
    if (current_lr_arm_motor_q != nullptr) {
        init_data_ = *current_lr_arm_motor_q;
    }

    // #TODO: Get current camera/lidar frame to get reference for desired transforms
    #ifdef USE_CASADI
        
        // Set optimization initial guess and parameters
        opti_.set_initial(var_q_, eigen_to_casadi(init_data_));
        opti_.set_value(param_tf_l_, eigen_to_casadi(left_wrist));
        opti_.set_value(param_tf_r_, eigen_to_casadi(right_wrist));
        opti_.set_value(var_q_last_, eigen_to_casadi(init_data_));

        std::cout << "[G1_29_ArmIK] >>> Solving IK with CasADi..." << std::endl;
    
    #else // USE_CASADI

        var_q_ = init_data_;
        param_tf_l_ = eigen_to_pinocchio(left_wrist);
        param_tf_r_ = eigen_to_pinocchio(right_wrist);
        var_q_last_ = init_data_;


        Eigen::VectorXd v_itr(reduced_model_.nv);
        pinocchio::Data::Matrix6x J_left(6,reduced_model_.nv);
        pinocchio::Data::Matrix6x J_right(6,reduced_model_.nv);
        pinocchio::Data::MatrixXs J(12,reduced_model_.nv);
        J_left.setZero();
        J_right.setZero();

        // Frames for camera and lidar
        // reduced_data_.oMf[oMcamera];
        // reduced_data_.oMf[oMLidar];

    #endif // USE_CASADI

    try {

        #ifdef USE_CASADI
            std::cout << "[G1_29_ArmIK] >>> Starting optimization..." << std::endl;
            // Solve optimization problem
            casadi::OptiSol sol = opti_.solve();
            
            // Extract solution
            std::vector<double> sol_q_vec = static_cast<std::vector<double>>(sol.value(var_q_));

            std::cout << "[G1_29_ArmIK] >>> IK Solution Found: ";
            for (auto a: sol_q_vec) {
                std::cout << a << ", ";
            }
            std::cout << std::endl;
        #else // USE_CASADI
            // Using interation method
            for (int i=0;;i++)
            {
                pinocchio::forwardKinematics(reduced_model_,reduced_data_,var_q_);
                const pinocchio::SE3 left_dMi = param_tf_l_.actInv(reduced_data_.oMi[left_wrist_id_]);
                const pinocchio::SE3 right_dMi = param_tf_r_.actInv(reduced_data_.oMi[right_wrist_id_]);
                err.head<6>() = pinocchio::log6(left_dMi).toVector();
                err.tail<6>() = pinocchio::log6(right_dMi).toVector();

                if(err.norm() < eps) {
                    break;
                }
                if (i >= IT_MAX) {
                    throw std::runtime_error("Maximum iterations reached without convergence.");
                }
                pinocchio::computeJointJacobian(reduced_model_,reduced_data_,var_q_,left_wrist_id_,J_left);
                pinocchio::computeJointJacobian(reduced_model_,reduced_data_,var_q_,right_wrist_id_,J_right);
                J.topRows<6>() = J_left;
                J.bottomRows<6>() = J_right;
                pinocchio::Data::MatrixXs JJt;
                JJt.noalias() = J * J.transpose();
                JJt.diagonal().array() += damp;
                v_itr.noalias() = - J.transpose() * JJt.ldlt().solve(err);
                var_q_ = pinocchio::integrate(reduced_model_,var_q_,v_itr * DT);
            }

            var_q_last_ = var_q_;

            // Extract solution
            std::vector<double> sol_q_vec(var_q_.data(), var_q_.data() + var_q_.size());
        #endif // USE_CASADI

        Eigen::VectorXd sol_q = Eigen::Map<Eigen::VectorXd>(sol_q_vec.data(), reduced_model_.nq);
        
        // Apply smoothing filter
        smooth_filter_->add_data(sol_q);
        sol_q = smooth_filter_->filtered_data();
        
        // Compute velocity
        Eigen::VectorXd v;
        if (current_lr_arm_motor_dq != nullptr) {
            v = (*current_lr_arm_motor_dq) * 0.0;
        } else {
            v = (sol_q - init_data_) * 0.0;
        }
        
        init_data_ = sol_q;
        
        // Compute feedforward torques using RNEA
        Eigen::VectorXd sol_tauff = pinocchio::rnea(
            reduced_model_, reduced_data_, sol_q, v,
            Eigen::VectorXd::Zero(reduced_model_.nv)
        );

        JointState result;

        std::cout << "IK Solved for: ";
        for (int joint_id = 1; joint_id <= reduced_model_.nv; ++joint_id) {
            result.name.push_back(reduced_model_.names[joint_id]);
            result.position.push_back(sol_q[reduced_model_.idx_qs[joint_id]]);
            result.velocity.push_back(v[reduced_model_.idx_vs[joint_id]]);
            result.effort.push_back(sol_tauff[reduced_model_.idx_vs[joint_id]]);
        }
        
        return result;
        
    } catch (const std::exception& e) {
        std::cerr << "ERROR in convergence: " << e.what() << std::endl;
        
        #ifdef USE_CASADI
        // Get debug solution
        std::vector<double> sol_q_vec = static_cast<std::vector<double>>(
            opti_.debug().value(var_q_));
        #else // USE_CASADI
        // Get debug solution
        std::vector<double> sol_q_vec(var_q_.data(), var_q_.data() + var_q_.size());
        #endif // USE_CASADI

        Eigen::VectorXd sol_q = Eigen::Map<Eigen::VectorXd>(
            sol_q_vec.data(), reduced_model_.nq);
        
        smooth_filter_->add_data(sol_q);
        sol_q = smooth_filter_->filtered_data();
        
        Eigen::VectorXd v;
        if (current_lr_arm_motor_dq != nullptr) {
            v = (*current_lr_arm_motor_dq) * 0.0;
        } else {
            v = (sol_q - init_data_) * 0.0;
        }
        
        init_data_ = sol_q;
        
        Eigen::VectorXd sol_tauff = pinocchio::rnea(
            reduced_model_, reduced_data_, sol_q, v,
            Eigen::VectorXd::Zero(reduced_model_.nv)
        );
        
        std::cerr << "sol_q: " << sol_q.transpose() << std::endl;
        std::cerr << "left_pose:\n" << left_wrist << std::endl;
        std::cerr << "right_pose:\n" << right_wrist << std::endl;

        JointState result;

        for (int joint_id = 0; joint_id < reduced_model_.njoints; ++joint_id) {
            result.name.push_back(reduced_model_.names[joint_id]);
            result.position.push_back(sol_q[reduced_model_.idx_qs[joint_id]]);
            result.velocity.push_back(v[reduced_model_.idx_vs[joint_id]]);
            result.effort.push_back(sol_tauff[reduced_model_.idx_vs[joint_id]]);
        }
        return result;
    }
}

} // namespace IK
