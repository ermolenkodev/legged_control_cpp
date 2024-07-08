#include "legged_control_cpp/forward_kinematics.hpp"
#include "iostream"
#include "legged_control_cpp/model.hpp"
#include "legged_control_cpp/spatial.hpp"
#include "legged_control_cpp/type_aliases.hpp"
#include "legged_control_cpp/utilities.hpp"
#include <stdexcept>
#include <variant>

namespace legged_ctrl {

namespace details {
  Placement extract_placement(SpatialMatrix const &X);
}

LCC_DISABLE_SIGN_CONVERSION
Placement compute_end_effector_placement(MultibodyModel const &model, VectorX const &q)
{
  auto const &[name, n_bodies, joints, parent, X_tree, I, _] = model;

  SpatialMatrix Xup{ SpatialMatrix::Zero() };
  SpatialMatrix wXi = SpatialMatrix::Identity();

  for (int i = 0; i < n_bodies; ++i) {
    double const qi = q(i);
    auto Xj = std::visit([qi](auto &&joint) { return joint.joint_transform(qi); }, joints[i]);
    Xup = Xj * X_tree[i];
    wXi = wXi * Xup.inverse();
  }

  auto nTee = model.get_ee_transform();
  if (!nTee.has_value()) { throw std::runtime_error("No end effector placement"); }
  auto nXee = Ad(nTee.value());
  SpatialMatrix const wXee = wXi * nXee;

  return details::extract_placement(wXee);
}

KinematicsResult compute_end_effector_forward_kinematics(MultibodyModel const &model,
  SystemConfiguration const &system_configuration,
  ReferenceFrame const reference_frame)
{
  if (!model.get_ee_transform()) { throw std::runtime_error("No end effector placement"); }

  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  SE3 const &nTee = model.get_ee_transform().value();
  auto const &[name, n_bodies, joints, parent, X_tree, I, _] = model;

  SpatialMatrix Xup{ SpatialMatrix::Zero() };
  SpatialMatrix wXi{ SpatialMatrix::Identity() };
  SpatialVector S{ SpatialVector::Zero() };

  IndexedSpatialVectors V{ { -1, SpatialVector::Zero() } };
  IndexedSpatialVectors A{ { -1, SpatialVector::Zero() } };

  auto const &[q, qd, qdd] = system_configuration;

  for (int i = 0; i < n_bodies; ++i) {
    double const qi = q(i);
    auto Xj = std::visit([qi](auto &&joint) { return joint.joint_transform(qi); }, joints[i]);
    Xup = Xj * X_tree[i];
    wXi = wXi * Xup.inverse();
    S = std::visit([](auto &&joint) { return joint.screw_axis(); }, joints[i]);
    auto Vj = S * qd(i);
    V[i] = Xup * V[parent[i]] + Vj;
    A[i] = Xup * A[parent[i]] + S * qdd(i) + Vx(V[i]) * Vj;
  }

  SpatialVector const &nV = V.at(model.n_bodies - 1);
  SpatialVector const &nA = A.at(model.n_bodies - 1);

  SpatialMatrix const nXee = Ad(nTee);
  SpatialMatrix const wXee = wXi * nXee;

  if (reference_frame == ReferenceFrame::WORLD) { return { wXi * nV, wXi * nA }; }

  if (reference_frame == ReferenceFrame::LOCAL_WORLD_ALIGNED) {
    SpatialMatrix const translation = translation_part(wXee);
    return { translation.inverse() * (wXi * nV), translation.inverse() * (wXi * nA) };
  }

  throw std::runtime_error("Unknown reference frame");// Should never reach this point
}

SpatialVector compute_end_effector_classical_acceleration(MultibodyModel const &model,
  SystemConfiguration const &system_configuration,
  ReferenceFrame const reference_frame)
{
  auto [V, A] = compute_end_effector_forward_kinematics(model, system_configuration, reference_frame);

  auto const &v = V.template bottomRows<3>();
  auto const &omega = V.template topRows<3>();

  A.template bottomRows(3) += omega.cross(v);

  return A;
}
LCC_END_DISABLE_WARNINGS


Placement details::extract_placement(SpatialMatrix const &X)
{
  SO3 const &R = X.template topLeftCorner<3, 3>();
  SkewSymmetric const translation_so3 = translation_part(X).bottomLeftCorner<3, 3>();

  return { vector_from_SO3(translation_so3), R };
}

}// namespace legged_ctrl
