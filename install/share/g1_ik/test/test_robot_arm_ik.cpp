#include <arm_ik/robot_arm_ik.h>

int main(){

    #ifdef USE_CASADI
    std::cout << "Using CasADi for optimization." << std::endl;
    #else
    std::cout << "Not using CasADi for optimization." << std::endl;
    #endif

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

        #include <chrono>

        auto start = std::chrono::high_resolution_clock::now();
        auto [q, tau] = ik_solver.solve_ik(left_target, right_target);
        auto end = std::chrono::high_resolution_clock::now();

        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        std::cout << "solve_ik time: " << elapsed_us << " us" << std::endl;

        std::cout << "IK solution q: " << q.transpose() << std::endl;
        std::cout << "IK solution tau: " << tau.transpose() << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception during IK solving: " << e.what() << std::endl;
        return -1;
    }
}