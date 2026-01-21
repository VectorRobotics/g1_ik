#include <arm_ik/robot_arm_ik.h>

int main(){

    try{
        auto ik_solver = G1_29_ArmIK();
    }
    catch (const std::exception& e) {
        std::cerr << "Exception during IK solver initialization: " << e.what() << std::endl;
        return -1;
    }

    try{
        Eigen::Matrix4d left_target = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d right_target = Eigen::Matrix4d::Identity();

        auto ik_solver = G1_29_ArmIK();

        auto [q, tau] = ik_solver.solve_ik(left_target, right_target);

        std::cout << "IK solution q: " << q.transpose() << std::endl;
        std::cout << "IK solution tau: " << tau.transpose() << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception during IK solving: " << e.what() << std::endl;
        return -1;
    }
}