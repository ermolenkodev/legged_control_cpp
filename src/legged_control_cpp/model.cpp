#include "legged_control_cpp/model.hpp"
#include "legged_control_cpp/builder.hpp"

namespace legged_ctrl {

MultibodyModelBuilderWithoutRoot MultibodyModel::create() { return MultibodyModelBuilderWithoutRoot{}; }

std::ostream &operator<<(std::ostream &os, MultibodyModel const &model)
{
  os << model.name_ << std::endl;
  os << "n_bodies: " << model.n_bodies_ << std::endl;
  return os;
}

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

}// namespace legged_ctrl
