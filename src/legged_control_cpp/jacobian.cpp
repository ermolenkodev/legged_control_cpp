#include "legged_control_cpp/jacobian.hpp"
#include "legged_control_cpp/utilities.hpp"

namespace legged_ctrl {

LCC_DISABLE_SIGN_CONVERSION
JacobianMatrix compute_end_effector_frame_jacobian(MultibodyModel const &model,
  VectorX const &q,
  ReferenceFrame const reference_frame)
{
  auto const &[name, n_bodies, joints, parent, X_tree, I, _] = model;
  JacobianMatrix J = JacobianMatrix::Zero(SIX_DIM, n_bodies);

  std::vector<SpatialMatrix> Xup(n_bodies, SpatialMatrix::Zero());
  IndexedSpatialMatrices wXi = { { -1, SpatialMatrix ::Identity() } };

  for (int i = 0; i < n_bodies; ++i) {
    double const qi = q(i);
    auto Xj = std::visit([qi](auto &&joint) { return joint.joint_transform(qi); }, joints[i]);
    Xup[i] = Xj * X_tree[i];
    wXi[i] = wXi[parent[i]] * Xup[i].inverse();
    SpatialVector const S = std::visit([](auto &&joint) { return joint.screw_axis(); }, joints[i]);
    J.col(i) = wXi[i] * S;
  }

  if (reference_frame == ReferenceFrame::LOCAL_WORLD_ALIGNED) {
    auto nTee = model.get_ee_transform();
    if (!nTee.has_value()) { throw std::runtime_error("No end effector placement"); }
    auto nXee = Ad(nTee.value());
    SpatialMatrix const wXee = wXi[n_bodies - 1] * nXee;
    SpatialMatrix const translation = translation_part(wXee);

    return translation.inverse() * J;
  }

  return J;
}
LCC_END_DISABLE_WARNINGS

}// namespace legged_ctrl
