#include "legged_control_cpp/builder.hpp"

#include "fmt/core.h"
#include "legged_control_cpp/type_aliases.hpp"
#include <utility>

namespace legged_ctrl {
MultibodyModelBuilderBase::MultibodyModelBuilderBase(ModelPtr model_, LinkNameToIndexMapPtr link_name_to_idx_)
  : model{ std::move(model_) }, link_name_to_idx{ std::move(link_name_to_idx_) }
{}

MultibodyModelBuilderWithoutRoot::MultibodyModelBuilderWithoutRoot()
  : MultibodyModelBuilderBase{ std::make_shared<MultibodyModel>(), std::make_shared<LinkNameToIndexMap>() }
{}

MultibodyModelBuilder::MultibodyModelBuilder(ModelPtr model, LinkNameToIndexMapPtr link_name_to_idx)
  : MultibodyModelBuilderBase(std::move(model), std::move(link_name_to_idx))
{}

MultibodyModelBuilder MultibodyModelBuilderWithoutRoot::set_root(const urdf::ModelInterfaceSharedPtr &urdf_tree)
{
  model->name = urdf_tree->getName();
  ::urdf::LinkConstSharedPtr const root = urdf_tree->getRoot();
  (*link_name_to_idx)[root->name] = -1;

  return { model, link_name_to_idx };
}

constexpr double EPSILON = 1e-9;

bool operator==(::urdf::Vector3 const &lhs, urdf::Vector3 const &rhs)
{
  auto close_enough = [](double a, double b) { return std::abs(a - b) < EPSILON; };
  return close_enough(lhs.x, rhs.x) && close_enough(lhs.y, rhs.y) && close_enough(lhs.z, rhs.z);
}

JointAxis extract_joint_axis(const ::urdf::Vector3 &axis)
{
  using AxisVec = std::tuple<::urdf::Vector3, JointAxis>;

  std::array<AxisVec, 3> const axis_mappings{ std::make_tuple(::urdf::Vector3(1., 0., 0.), JointAxis::X),
    std::make_tuple(::urdf::Vector3(0., 1., 0.), JointAxis::Y),
    std::make_tuple(::urdf::Vector3(0., 0., 1.), JointAxis::Z) };

  for (auto const &[vec, joint_axis] : axis_mappings) {
    if (axis == vec) { return joint_axis; }
  }

  return JointAxis::UNALIGNED;
}

SE3 convertFromUrdf(::urdf::Pose const &pose)
{
  ::urdf::Vector3 const &p = pose.position;
  ::urdf::Rotation const &q = pose.rotation;
  SE3 R;
  R.topLeftCorner<3, 3>() = Quaternion(q.w, q.x, q.y, q.z).matrix();
  R.topRightCorner<3, 1>() = Vector3(p.x, p.y, p.z);
  R(3, 3) = 1.0;

  return R;
}

void not_implemented() { throw std::runtime_error("Function not yet implemented"); }

MultibodyModelBuilder &MultibodyModelBuilder::add_link(::urdf::LinkConstSharedPtr const &link)
{
  std::cout << "\n\n";
  std::cout << "Processing link: " << link->name << "\n";

  if (link->inertial == nullptr) {
    std::cout << "Link " << link->name << " has no inertial, skipping"
              << "\n";
    return *this;
  }

  std::string const &parent_link_name = link->getParent()->name;
  if (link_name_to_idx->find(parent_link_name) == link_name_to_idx->end()) {
    throw std::invalid_argument(fmt::format(
      "Failed to process link: {}. Parent link {} not found, it must be added before", link->name, parent_link_name));
  }
  int const parent_idx = (*link_name_to_idx)[parent_link_name];
  model->parent.emplace_back(parent_idx);
  (*link_name_to_idx)[link->name] = static_cast<int>((*model).parent.size());
  model->n_bodies++;

  if (link->parent_joint->type == ::urdf::Joint::REVOLUTE) {
    model->joints.emplace_back(RevoluteJoint(extract_joint_axis(link->parent_joint->axis)));
  } else {
    not_implemented();
  }

  auto X = Ad(convertFromUrdf(link->parent_joint->parent_to_joint_origin_transform));
  std::cout << "Spatial transform"
            << "\n";
  std::cout << X << "\n";
  model->X_tree.emplace_back(X);

  ::urdf::Inertial const &Y = *link->inertial;
  ::urdf::Vector3 const &p = Y.origin.position;
  ::urdf::Rotation const &q = Y.origin.rotation;

  Vector3 p_C_i = Vector3(p.x, p.y, p.z);
  SO3 const &R_i_C = Quaternion(q.w, q.x, q.y, q.z).matrix();
  std::cout << "CoM position"
            << "\n";
  std::cout << p_C_i << "\n";
  std::cout << "CoM orientation"
            << "\n";
  std::cout << R_i_C << "\n";

  RotationalInertia rotI_C_C;
  rotI_C_C << Y.ixx, Y.ixy, Y.ixz, Y.ixy, Y.iyy, Y.iyz, Y.ixz, Y.iyz, Y.izz;
  std::cout << "Rotation inertia in CoM frame"
            << "\n";
  std::cout << rotI_C_C << "\n";

  // why could not infer type argument without it
  RotationalInertia rotI_C_i = R_i_C * rotI_C_C * R_i_C.transpose();
  std::cout << "Rotation inertia in link frame"
            << "\n";
  std::cout << rotI_C_i << "\n";

  auto I = I_from_rotinertia_about_com(rotI_C_i, p_C_i, Y.mass);
  std::cout << "Spatial inertia"
            << "\n";
  std::cout << I << "\n";
  // why could not infer type argument
  model->I.emplace_back(I);

  return *this;
}
}// namespace legged_ctrl