#include "legged_control_cpp/inverse_dynamics.hpp"
#include "legged_control_cpp/external_forces.hpp"

namespace legged_ctrl {

#pragma clang diagnostic push
#pragma GCC diagnostic push
#pragma clang diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"

RNEAExtendedOutput details::rnea_with_extended_output(const MultibodyModel &model,
  const VectorX &q,
  const VectorX &qd,
  const VectorX &qdd,
  bool skipCoriolisEffects,
  const ExternalForces &externalForces,
  const Vector3 &gravity)
{
  auto const &[name, n_bodies, joints, parent, X_tree, I, _] = model;

  // velocity of the base is zero
  IndexedSpatialVectors V{ { -1, SpatialVector::Zero() } };

  // Note: we are assign acceleration of the base to the -gravity,
  // but it is just a way to incorporate gravity term to the recursive force propagation formula
  IndexedSpatialVectors A{ { -1, -spatial_gravity(gravity) } };

  IndexedSpatialVectors F = { { -1, SpatialVector::Zero() } };
  std::vector<SpatialMatrix> Xup(n_bodies, SpatialMatrix::Zero());
  std::vector<SpatialVector> S(n_bodies, SpatialVector::Zero());

  // All indexed from 0
  for (int i = 0; i < n_bodies; ++i) {
    double const qi = q(i);
    auto Xj = std::visit([qi](auto &&joint) { return joint.joint_transform(qi); }, joints[i]);
    S[i] = std::visit([](auto &&joint) { return joint.screw_axis(); }, joints[i]);
    Xup[i] = Xj * X_tree[i];
    auto Vj = S[i] * qd(i);
    V[i] = Xup[i] * V[parent[i]] + Vj;
    A[i] = Xup[i] * A[parent[i]] + S[i] * qdd(i) + Vx(V[i]) * Vj;
    if (skipCoriolisEffects) {
      F[i] = I[i] * A[i];
    } else {
      F[i] = I[i] * A[i] + Vx_star(V[i]) * I[i] * V[i];
    }
  }

  apply_end_effector_exerted_force(F, externalForces, model);
  apply_external_forces(F, externalForces, model, Xup);

  VectorX tau{ n_bodies };
  for (int i = n_bodies - 1; i >= 0; --i) {
    tau(i) = S[i].transpose() * F[i];
    F[parent[i]] += Xup[i].transpose() * F[i];
  }

  return RNEAExtendedOutput{ V, A, F, Xup, S, tau };
}

#pragma clang diagnostic pop
#pragma GCC diagnostic pop

RNEAExtendedOutput rnea_with_extended_output(MultibodyModel const &model,
  SystemConfiguration const &systemConfiguration,
  ExternalForces const &externalForces,
  Vector3 const &gravity)
{
  return details::rnea_with_extended_output(
    model, systemConfiguration.q, systemConfiguration.qd, systemConfiguration.qdd, false, externalForces, gravity);
}

VectorX rnea(MultibodyModel const &model,
  SystemConfiguration const &systemConfiguration,
  ExternalForces const &externalForces,
  Vector3 const &gravity)
{
  RNEAExtendedOutput const output = rnea_with_extended_output(model, systemConfiguration, externalForces, gravity);
  return output.tau();
}

VectorX RNEAExtendedOutput::tau() const { return manipulator_eq_lhs_; }

VectorX RNEAExtendedOutput::nle() const { return manipulator_eq_lhs_; }

VectorX RNEAExtendedOutput::g() const { return manipulator_eq_lhs_; }


}// namespace legged_ctrl
