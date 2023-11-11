#ifndef LCC_EXTERNAL_FORCES_HPP
#define LCC_EXTERNAL_FORCES_HPP

#include "inverse_dynamics.hpp"
#include "model.hpp"
#include "type_aliases.hpp"

namespace legged_ctrl {

void apply_end_effector_exerted_force(IndexedSpatialVectors &F,
  ExternalForces const &external_forces,
  MultibodyModel const &model);

void apply_external_forces(IndexedSpatialVectors &F,
  ExternalForces const &external_forces,
  MultibodyModel const &model,
  std::vector<SpatialMatrix> const &Xup);

}// namespace legged_ctrl

#endif
