#ifndef LCC_GAINS_HPP
#define LCC_GAINS_HPP

#include "legged_control_cpp/type_aliases.hpp"

namespace task_space_control {

using namespace legged_ctrl;


struct GainMatrices
{
  DiagonalMatrix Kp_pos;
  DiagonalMatrix Kd_pos;
  DiagonalMatrix Kp_orient;
  DiagonalMatrix Kd_orient;

  GainMatrices() : Kp_pos(3), Kd_pos(3), Kp_orient(3), Kd_orient(3)
  {
    Kp_pos.diagonal() = legged_ctrl::Vector3{ 500, 500, 500 };// NOLINT
    Kd_pos.diagonal() = legged_ctrl::Vector3{ 50, 50, 50 };// NOLINT
    Kp_orient.diagonal() = legged_ctrl::Vector3{ 10, 10, 10 };// NOLINT
    Kd_orient.diagonal() = legged_ctrl::Vector3{ .1, .1, .1 };// NOLINT
  }
};

}// namespace task_space_control

#endif
