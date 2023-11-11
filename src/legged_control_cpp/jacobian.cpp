#include "legged_control_cpp/jacobian.hpp"

namespace legged_ctrl {

#pragma clang diagnostic push
#pragma GCC diagnostic push
#pragma clang diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"

JacobianMatrix compute_end_effector_frame_jacobian(MultibodyModel const &model, VectorX const &q, ReferenceFrame const reference_frame)
{
  auto const &[name, n_bodies, joints, parent, X_tree, I, _] = model;
  JacobianMatrix J = JacobianMatrix::Zero(SIX_DIM, n_bodies);

  std::vector<SpatialMatrix> Xup(n_bodies, SpatialMatrix::Zero());
  IndexedSpatialMatrices wXi = { { -1, SpatialMatrix ::Identity() } };

  for (int i = 0; i < n_bodies; ++i) {
    double const qi = q(i);
    auto Xj = std::visit([qi](auto &&joint) { return joint.joint_transform(qi); }, joints[i]);
    Xup[i] = Xj * X_tree[i];
    std::cout << "Xup[" << i << "]: " << std::endl << Xup[i] << std::endl;
    wXi[i] = wXi[parent[i]] * Xup[i].inverse();
    std::cout << "wXi[" << i << "]: " << std::endl << wXi[i] << std::endl;
    SpatialVector const S = std::visit([](auto &&joint) { return joint.screw_axis(); }, joints[i]);
    J.col(i) = wXi[i] * S;
    std::cout << "J.col(" << i << "): " << std::endl << J.col(i) << std::endl;
  }

  std::cout << "J: " << std::endl << J << std::endl;

  std::cout << "wXi[n_bodies - 1]: " << std::endl << wXi[n_bodies - 1] << std::endl;

  if (reference_frame == ReferenceFrame::LOCAL_WORLD_ALIGNED) {
    auto nTee = model.get_ee_transform();
    std::cout << "nTee: " << std::endl << nTee.value() << std::endl;
    if (!nTee.has_value()) { throw std::runtime_error("No end effector placement"); }
    auto nTee_trans = translation_part(nTee.value());
    std::cout << "nTee transl: " << std::endl << nTee_trans << std::endl;
    auto nXee = Ad(nTee.value());
    std::cout << "nXee: " << std::endl << nXee << std::endl;
    SpatialMatrix const wXee = wXi[n_bodies - 1] * nXee;
    std::cout << "wXee: " << std::endl << wXee << std::endl;
    SpatialMatrix const translation = translation_part(wXee);
     std::cout << "translation: " << std::endl << translation << std::endl;

    return translation.inverse() * J;
  }

  return J;
}

#pragma clang diagnostic pop
#pragma GCC diagnostic pop

}// namespace legged_ctrl
