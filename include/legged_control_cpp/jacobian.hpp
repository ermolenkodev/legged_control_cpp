#ifndef LCC_JACOBIAN_H
#define LCC_JACOBIAN_H

#include "model.hpp"
#include "refernce_frame.hpp"
#include "type_aliases.hpp"

namespace legged_ctrl {

JacobianMatrix compute_end_effector_frame_jacobian(MultibodyModel const &model,
  VectorX const &q,
  ReferenceFrame const reference_frame = ReferenceFrame::WORLD);

}// namespace legged_ctrl

#endif