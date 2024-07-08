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
  double Kp_posture;
  double Kd_posture;

  GainMatrices(double Kpp, double Kdp, double Kpo, double Kdo, double Kp_posture_, double Kd_posture_)
    : Kp_pos(3), Kd_pos(3), Kp_orient(3), Kd_orient(3), Kp_posture(Kp_posture_), Kd_posture(Kd_posture_)
  {
    Kp_pos.diagonal() = legged_ctrl::Vector3{ Kpp, Kpp, Kpp };// NOLINT
    Kd_pos.diagonal() = legged_ctrl::Vector3{ Kdp, Kdp, Kdp };// NOLINT
    Kp_orient.diagonal() = legged_ctrl::Vector3{ Kpo, Kpo, Kpo };// NOLINT
    Kd_orient.diagonal() = legged_ctrl::Vector3{ Kdo, Kdo, Kdo };// NOLINT
  }
};

}// namespace task_space_control

#endif
