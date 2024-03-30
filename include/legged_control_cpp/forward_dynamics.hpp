#ifndef LCC_FORWARD_DYNAMICS_H
#define LCC_FORWARD_DYNAMICS_H

#include "functional"
#include "model.hpp"
#include "type_aliases.hpp"

namespace legged_ctrl {

struct CRBAResult {
  JointSpaceMatrix mass_matrix_;
  VectorX nle_;

  CRBAResult(JointSpaceMatrix mass_matrix, VectorX nle);
};

JointSpaceMatrix compute_mass_matrix_crba(MultibodyModel const &model,
  std::vector<SpatialMatrix> const &Xup,
  std::vector<SpatialVector> const &S,
  std::vector<SpatialMatrix> const &Ic);

CRBAResult crba(MultibodyModel const &model,
  SystemConfiguration const &system_configuration,
  ExternalForces const &external_forces = ExternalForces::none(),
  Vector3 const &gravity = Vector3 { 0, 0, G });

std::vector<SpatialMatrix> compute_composite_inertia(const MultibodyModel &model,
  const std::vector<SpatialMatrix> &Xup);

VectorX compute_gravity_effect(MultibodyModel const &model,
  SystemConfiguration const &system_configuration,
  ExternalForces const &external_forces = ExternalForces::none(),
  Vector3 const &gravity = Vector3 { 0, 0, G });

}// namespace legged_ctrl

#endif// LCC_FORWARD_DYNAMICS_H
