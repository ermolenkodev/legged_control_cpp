#ifndef TYPE_ALIASES_HPP
#define TYPE_ALIASES_HPP

#include "Eigen/Core"
#include <Eigen/Geometry>

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
}

#endif