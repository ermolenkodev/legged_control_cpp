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
  os << model.name_ << std::endl;
  os << "n_bodies: " << model.n_bodies_ << std::endl;
  return os;
}

LCC_DISABLE_SIGN_CONVERSION
bool MultibodyModel::operator==(const MultibodyModel &other) const
{
  if (n_bodies_ != other.n_bodies_) { return false; }

  for (int i = 0; i < n_bodies_; ++i) {
    if (joints_[i] != other.joints_[i]) { return false; }
    if (parent_[i] != other.parent_[i]) { return false; }
    if (!X_tree_[i].isApprox(other.X_tree_[i])) { return false; }
    if (!I_[i].isApprox(other.I_[i])) { return false; }
  }

  return true;
}
LCC_END_DISABLE_WARNINGS

}// namespace legged_ctrl
