#ifndef PINOCCHIO_EIGEN_UTILS_H
#define PINOCCHIO_EIGEN_UTILS_H

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/spatial/se3.hpp>


// /**
//  * @brief Convert Eigen::VectorXd to pinocchio::SE3
//  */
// inline pinocchio::SE3 eigen_to_pinocchio(const Eigen::VectorXd& vec) {
//     std::vector<double> data(vec.data(), vec.data() + vec.size());
//     return pinocchio::VectorXb(data);
// }

/**
 * @brief Convert Eigen::Matrix4d to pinocchio::SE3
 */
inline pinocchio::SE3 eigen_to_pinocchio(const Eigen::Matrix4d& mat) {
    return pinocchio::SE3(mat);
}

// /**
//  * @brief Convert Eigen::MatrixXd to pinocchio::DM
//  */
// inline pinocchio::DM eigen_to_pinocchio(const Eigen::MatrixXd& mat) {
//     std::vector<double> data;
//     data.reserve(16);
//     for (int col = 0; col < 4; ++col) {
//         for (int row = 0; row < 4; ++row) {
//             data.push_back(mat(row, col));
//         }
//     }
//     return pinocchio::DM::reshape(pinocchio::DM(data), 4, 4);
// }

// /**
//  * @brief Convert pinocchio::DM to Eigen::VectorXd
//  */
// inline Eigen::VectorXd pinocchio_to_eigen_vector(const pinocchio::DM& dm) {
//     std::vector<double> data = static_cast<std::vector<double>>(dm);
//     return Eigen::Map<Eigen::VectorXd>(data.data(), data.size());
// }

// /**
//  * @brief Convert pinocchio::DM to Eigen::MatrixXd
//  */
// inline Eigen::MatrixXd pinocchio_to_eigen_matrix(const pinocchio::DM& dm) {
//     std::vector<double> data = static_cast<std::vector<double>>(dm);
//     int rows = dm.rows();
//     int cols = dm.columns();
    
//     Eigen::MatrixXd mat(rows, cols);
//     for (int col = 0; col < cols; ++col) {
//         for (int row = 0; row < rows; ++row) {
//             mat(row, col) = data[col * rows + row];
//         }
//     }
//     return mat;
// }

#endif // PINOCCHIO_EIGEN_UTILS_H