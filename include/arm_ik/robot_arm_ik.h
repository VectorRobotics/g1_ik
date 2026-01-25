#ifndef ROBOT_ARM_IK_H
#define ROBOT_ARM_IK_H

#ifdef USE_CASADI
    #include "casadi_eigen_utils.h"
#endif // USE_CASADI

#include "pinocchio_eigen_utils.h"

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "weighted_moving_filter.h"

namespace IK {
/**
 * @brief Base configuration structure for robot models
 */
struct RobotConfig {
    std::string asset_file;
    std::string asset_root;
};

/**
 * @brief Return Type for IK.
 */
struct JointState {
    std::vector<std::string> name;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
};



/**
 * @brief G1_29_ArmIK - Inverse kinematics solver for G1 robot with 29 DOF
 */
class G1_29_ArmIK {
public:
    G1_29_ArmIK(
        bool unit_test = false, 
        bool visualization = false,
        const RobotConfig* robot_config = nullptr
    );
    virtual ~G1_29_ArmIK();

    /**
     * @brief Solve inverse kinematics for both arms
     * @param left_wrist Target pose for left wrist (4x4 homogeneous transformation)
     * @param right_wrist Target pose for right wrist (4x4 homogeneous transformation)
     * @param current_lr_arm_motor_q Current joint positions (optional)
     * @param current_lr_arm_motor_dq Current joint velocities (optional)
     * @return Pair of (joint_positions, joint_torques)
     */
    virtual JointState solve_ik(
        const Eigen::Matrix4d& left_wrist,
        const Eigen::Matrix4d& right_wrist,
        const Eigen::VectorXd* current_lr_arm_motor_q = nullptr,
        const Eigen::VectorXd* current_lr_arm_motor_dq = nullptr
    );

    /**
     * @brief Scale arm poses based on arm length ratio
     * @param human_left_pose Human left arm pose
     * @param human_right_pose Human right arm pose
     * @param human_arm_length Human arm length (default: 0.60m)
     * @param robot_arm_length Robot arm length (default: 0.75m)
     * @return Pair of scaled (left_pose, right_pose)
     */
    std::pair<Eigen::Matrix4d, Eigen::Matrix4d> scale_arms(
        const Eigen::Matrix4d& human_left_pose,
        const Eigen::Matrix4d& human_right_pose,
        double human_arm_length = 0.60,
        double robot_arm_length = 0.75
    );

protected:
    void setup_optimization();
    void initialize_joints_to_lock();
    void add_end_effector_frames();

    bool unit_test_;
    bool visualization_;
    RobotConfig robot_config_;
    std::string urdf_path_;
    std::string model_dir_;

    pinocchio::Model robot_model_;
    pinocchio::Data robot_data_;
    pinocchio::Model reduced_model_;
    pinocchio::Data reduced_data_;

    #ifdef USE_CASADI

        casadi::Opti opti_;
        casadi::MX var_q_;
        casadi::MX var_q_last_;
        casadi::MX param_tf_l_;
        casadi::MX param_tf_r_;

    #else // USE_CASADI

        const double eps  = 1e-4;
        const int IT_MAX  = 1000;
        const double DT   = 1e-1;
        const double damp = 1e-6;

        Eigen::VectorXd var_q_;
        Eigen::VectorXd var_q_last_;
        pinocchio::SE3 param_tf_l_;
        pinocchio::SE3 param_tf_r_;
        Eigen::Matrix<double, 12,1> err;

    #endif // USE_CASADI

    pinocchio::JointIndex left_wrist_id_;
    pinocchio::JointIndex right_wrist_id_;
    pinocchio::FrameIndex L_hand_id_;
    pinocchio::FrameIndex R_hand_id_;

    pinocchio::FrameIndex oMcamera;
    pinocchio::FrameIndex oMLidar;

    Eigen::VectorXd init_data_;
    std::unique_ptr<WeightedMovingFilter> smooth_filter_;

    std::vector<std::string> mixed_joints_to_lock_ids_;
};

} // namespace IK
#endif // ROBOT_ARM_IK_H