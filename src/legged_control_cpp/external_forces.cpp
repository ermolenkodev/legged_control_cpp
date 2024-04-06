#include "legged_control_cpp/external_forces.hpp"
#include "legged_control_cpp/utilities.hpp"

namespace legged_ctrl {

void apply_end_effector_exerted_force(IndexedSpatialVectors &F,
  ExternalForces const &external_forces,
  MultibodyModel const &model)
{
  if (F.empty()) { return; }
  if (!external_forces.f_tip.has_value()) { return; }
  auto T_n_ee = model.get_ee_transform();
  if (!T_n_ee.has_value()) { return; }
  auto &last_link_wrench = F.at(model.n_bodies_ - 1);
  auto X_n_ee = Ad(Tinv(T_n_ee.value()));
  last_link_wrench += X_n_ee.transpose() * external_forces.f_tip.value();
}

LCC_DISABLE_SIGN_CONVERSION
void apply_external_forces(IndexedSpatialVectors &F,
  ExternalForces const &external_forces,
  MultibodyModel const &model,
  std::vector<SpatialMatrix> const &Xup)
{
  if (F.empty()) { return; }
  if (!external_forces.f_ext.has_value()) { return; }
  if (external_forces.f_ext->empty()) { return; }

  auto const &f_ext = external_forces.f_ext.value();
  auto const &parent = model.get_parent_idxs();
  std::unordered_map<int, SpatialMatrix> X_i_0{ { -1, SpatialMatrix::Identity() } };

  for (int i = 0; i < model.num_bodies(); ++i) {
    X_i_0[i] = Xup[i] * X_i_0[parent[i]];
    if (f_ext.find(i) != f_ext.end()) {
      Eigen::FullPivLU<Eigen::MatrixXd> const lu(X_i_0[i].transpose());
      F[i] -= lu.solve(f_ext.at(i));
    }
  }
}
LCC_END_DISABLE_WARNINGS

}// namespace legged_ctrl
