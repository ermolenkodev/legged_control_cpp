#include "legged_control_cpp/forward_kinematics.hpp"
#include "legged_control_cpp/model.hpp"
#include "legged_control_cpp/spatial.hpp"
#include "legged_control_cpp/type_aliases.hpp"
#include <stdexcept>
#include <variant>

namespace legged_ctrl {

namespace details {
  Placement extract_placement(SpatialMatrix const &X);
}

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

Placement details::extract_placement(SpatialMatrix const &X)
{
  SO3 const &R = X.template topLeftCorner<3, 3>();
  SpatialMatrix const spatial_translation = translation_part(X);
  SkewSymmetric const translation_so3 = translation_part(X).bottomLeftCorner<3, 3>();

  return { vector_from_SO3(translation_so3), R };
}

}// namespace legged_ctrl
