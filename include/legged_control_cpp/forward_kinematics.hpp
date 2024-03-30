#ifndef LCC_FORWARD_KINEMATICS_HPP
#define LCC_FORWARD_KINEMATICS_HPP

#include "model.hpp"
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

}// namespace legged_ctrl

#endif// LCC_FORWARD_KINEMATICS_HPP