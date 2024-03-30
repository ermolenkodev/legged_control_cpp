#include "legged_control_cpp/forward_dynamics.hpp"
#include "legged_control_cpp/inverse_dynamics.hpp"

namespace legged_ctrl {

CRBAResult::CRBAResult(JointSpaceMatrix mass_matrix, VectorX nle)
  : mass_matrix_(std::move(mass_matrix)), nle_(std::move(nle))
{}

CRBAResult crba(MultibodyModel const &model,
  SystemConfiguration const &system_configuration,
  ExternalForces const &external_forces,
  Vector3 const &gravity)
{
  VectorX const qdd = VectorX::Zero(model.num_bodies());
  auto const &[V, A, F, Xup, S, C] = details::rnea_with_extended_output(
    model, system_configuration.q, system_configuration.qd, qdd, false, external_forces, gravity);

  std::vector<SpatialMatrix> const Ic = compute_composite_inertia(model, Xup);
  JointSpaceMatrix const mass_matrix = compute_mass_matrix_crba(model, Xup, S, Ic);

  return { mass_matrix, C };
}

#pragma clang diagnostic push
#pragma GCC diagnostic push
#pragma clang diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"

std::vector<SpatialMatrix> compute_composite_inertia(const MultibodyModel &model, const std::vector<SpatialMatrix> &Xup)
{
  int const n_bodies = model.num_bodies();
  std::vector<int> const &parent = model.get_parent_idxs();

  std::vector<SpatialMatrix> Ic(model.I_);
  for (int i = n_bodies - 1; i >= 0; --i) {
    if (parent[i] != -1) { Ic[parent[i]] += Xup[i].transpose() * Ic[i] * Xup[i]; }
  }

  return Ic;
}

JointSpaceMatrix compute_mass_matrix_crba(MultibodyModel const &model,
  std::vector<SpatialMatrix> const &Xup,
  std::vector<SpatialVector> const &S,
  std::vector<SpatialMatrix> const &Ic)
{
  int const n_bodies = model.num_bodies();
  std::vector<int> const &parent = model.get_parent_idxs();

  JointSpaceMatrix M = JointSpaceMatrix::Zero(n_bodies, n_bodies);
  for (int i = 0; i < n_bodies; ++i) {
    SpatialVector F = Ic[i] * S[i];
    M(i, i) = S[i].transpose() * F;
    int j = i;
    while (parent[j] != -1) {
      F = Xup[j].transpose() * F;
      j = parent[j];
      M(i, j) = S[j].transpose() * F;
      M(j, i) = M(i, j);
    }
  }

  return M;
}

VectorX compute_gravity_effect(const MultibodyModel &model,
  const SystemConfiguration &system_configuration,
  ExternalForces const &external_forces,
  Vector3 const &gravity)
{
  VectorX const qdd = VectorX::Zero(model.num_bodies());
  RNEAExtendedOutput const result = details::rnea_with_extended_output(
    model, system_configuration.q, system_configuration.qd, qdd, true, external_forces, gravity);

  return result.g();
}

#pragma clang diagnostic pop
#pragma GCC diagnostic pop

}// namespace legged_ctrl
