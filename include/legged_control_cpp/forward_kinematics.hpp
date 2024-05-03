#ifndef LCC_FORWARD_KINEMATICS_HPP
#define LCC_FORWARD_KINEMATICS_HPP

#include "model.hpp"
#include "refernce_frame.hpp"
#include "type_aliases.hpp"

namespace legged_ctrl {

struct Placement
{
  Vector3 position;
  Matrix3 orientation;

  Placement(Vector3 position, SO3 orientation) : position(std::move(position)), orientation(std::move(orientation)) {}

  explicit Placement(Vector3 position) : position(std::move(position)), orientation(SO3::Identity()) {}

  explicit Placement(SO3 orientation) : position(Vector3::Zero()), orientation(std::move(orientation)) {}
};

Placement compute_end_effector_placement(MultibodyModel const &model, VectorX const &q);

SpatialVector compute_end_effector_classical_acceleration(MultibodyModel const &model,
  SystemConfiguration const &system_configuration,
  ReferenceFrame const reference_frame = ReferenceFrame::WORLD);

struct KinematicsResult
{
  SpatialVector spatial_velocity;
  SpatialVector spatial_acceleration;
};

KinematicsResult compute_end_effector_forward_kinematics(MultibodyModel const &model,
  SystemConfiguration const &system_configuration,
  ReferenceFrame const reference_frame = ReferenceFrame::WORLD);


}// namespace legged_ctrl

#endif// LCC_FORWARD_KINEMATICS_HPP