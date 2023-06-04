#include "legged_control_cpp/model.hpp"
#include "legged_control_cpp/builder.hpp"

namespace legged_ctrl {
MultibodyModelBuilderWithoutRoot MultibodyModel::create() { return MultibodyModelBuilderWithoutRoot{}; }

std::ostream &operator<<(std::ostream &os, MultibodyModel const &model)
{
  os << model.name << std::endl;
  os << "n_bodies: " << model.n_bodies << std::endl;
  return os;
}
}// namespace legged_ctrl