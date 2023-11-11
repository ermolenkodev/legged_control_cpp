#ifndef TYPE_ALIASES_HPP
#define TYPE_ALIASES_HPP

#include "Eigen/Core"
#include "map"
#include "memory"
#include "spdlog/logger.h"
#include "spdlog/spdlog.h"
#include <Eigen/Geometry>

namespace legged_ctrl {
struct MultibodyModel;
}

namespace legged_ctrl {

constexpr int SIX_DIM = 6;
using SE3 = Eigen::Matrix4d;
using Quaternion = Eigen::Quaternion<double>;
using SpatialMatrix = Eigen::Matrix<double, SIX_DIM, SIX_DIM>;
using SpatialVector = Eigen::Matrix<double, SIX_DIM, 1>;
using SO3 = Eigen::Matrix3d;
using Vector3 = Eigen::Vector3d;
using SkewSymmetric = Eigen::Matrix3d;
using RotationalInertia = Eigen::Matrix3d;
using Matrix3 = Eigen::Matrix3d;
using VectorX = Eigen::VectorX<double>;
using IndexedSpatialVectors = std::unordered_map<int, SpatialVector>;
using LinkNameToIndexMap = std::map<std::string, int>;
using ModelPtr = std::shared_ptr<MultibodyModel>;
using LinkNameToIndexMapPtr = std::shared_ptr<LinkNameToIndexMap>;
using JointSpaceMatrix = Eigen::MatrixXd;
using JacobianMatrix = Eigen::MatrixXd;
using IndexedSpatialMatrices = std::unordered_map<int, SpatialMatrix>;

}// namespace legged_ctrl

#endif
