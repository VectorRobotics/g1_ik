#include <arm_ik/robot_arm_ik_g1_23dof.h>

using namespace IK;

// Helper function to create SE3 transformation matrix
Eigen::Matrix4d create_se3(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    transform.block<3, 1>(0, 3) = t;
    return transform;
}

// Helper function to create SE3 from quaternion components and translation
Eigen::Matrix4d create_se3(double qw, double qx, double qy, double qz, 
                           double tx, double ty, double tz) {
    Eigen::Quaterniond q(qw, qx, qy, qz);
    Eigen::Vector3d t(tx, ty, tz);
    return create_se3(q, t);
}


int main() {
    // Define target poses for left and right wrists
    // read translation (tx, ty, tz) for left and right targets from stdin
    double ltx = 0.8, lty = 0.3, ltz = 1.2;
    double rtx = 0.5, rty = 0.5, rtz = 1.2;
    // std::cout << "Enter left tx ty tz (or press Enter to use defaults 0.8 0.3 1.2): ";
    // if (!(std::cin >> ltx >> lty >> ltz)) {
    //     // no input -> keep defaults, and clear error state / remaining line
    //     std::cin.clear();
    //     std::string _dummy;
    //     std::getline(std::cin, _dummy);
    // }
    // std::cout << "Enter right tx ty tz (or press Enter to use defaults 0.5 0.5 1.2): ";
    // if (!(std::cin >> rtx >> rty >> rtz)) {
    //     std::cin.clear();
    //     std::string _dummy;
    //     std::getline(std::cin, _dummy);
    // }

    Eigen::Matrix4d left_target = create_se3(0.7071, 0, 0.7071, 0, ltx, lty, ltz);
    Eigen::Matrix4d right_target = create_se3(0.7071, 0, -0.7071, 0, rtx, rty, rtz);
    // Create robot configuration
    RobotConfig config;
    config.asset_file = "../assets/g1/g1_29dof_with_hand_rev_1_0.urdf";
    config.asset_root = "../assets/g1/";

    // Create IK solver with collision detection
    G1_29_ArmIK_NoWrists arm_ik(false, false, &config);

    // Define external forces (6D wrenches)
    Eigen::VectorXd ext_force_left = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd ext_force_right = Eigen::VectorXd::Zero(6);
    ext_force_left(2) = 5.0;  // 5N downward force

    // Solve IK with collision checking
    // time the IK solve
    #include <chrono>
    auto start = std::chrono::high_resolution_clock::now();
    auto result = arm_ik.solve_ik(
        left_target, right_target,
        nullptr, nullptr,  // current q, dq
        &ext_force_left, &ext_force_right,
        true  // enable collision checking
    );
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    auto q = Eigen::VectorXd::Map(result.position.data(), result.position.size());
    auto tau = Eigen::VectorXd::Map(result.effort.data(), result.effort.size());
    std::cout << "IK solve time: " << elapsed.count() << " s" << std::endl;

    std::cout << "IK solution q: " << q.transpose() << std::endl;
    std::cout << "IK solution tau: " << tau.transpose() << std::endl;

}