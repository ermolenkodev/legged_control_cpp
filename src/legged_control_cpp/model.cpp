#include "legged_control_cpp/model.hpp"
#include "legged_control_cpp/builder.hpp"
#include "legged_control_cpp/mjxml/mjxml_builder.hpp"
#include "legged_control_cpp/utilities.hpp"

namespace legged_ctrl {

MultibodyModelBuilderWithoutRoot MultibodyModel::create_from_urdf() { return MultibodyModelBuilderWithoutRoot{}; }

mjxml::MjxmlModelBuilderWithoutRoot MultibodyModel::create_from_mjxml()
{
  return mjxml::MjxmlModelBuilderWithoutRoot{};
}

std::ostream &operator<<(std::ostream &os, MultibodyModel const &model)
{
  os << model.name << std::endl;
  os << "n_bodies: " << model.n_bodies << std::endl;
  return os;
}

LCC_DISABLE_SIGN_CONVERSION
bool MultibodyModel::operator==(const MultibodyModel &other) const
{
  if (n_bodies != other.n_bodies) { return false; }

  for (int i = 0; i < n_bodies; ++i) {
    if (joints[i] != other.joints[i]) { return false; }
    if (parent[i] != other.parent[i]) { return false; }
    if (!X_tree[i].isApprox(other.X_tree[i])) { return false; }
    if (!I[i].isApprox(other.I[i])) { return false; }
  }

  return true;
}
LCC_END_DISABLE_WARNINGS

}// namespace legged_ctrl
