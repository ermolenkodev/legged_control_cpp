#include "legged_control_cpp/builder.hpp"

#include "fmt/core.h"
#include "legged_control_cpp/type_aliases.hpp"
#include "legged_control_cpp/utilities.hpp"
#include <utility>


namespace legged_ctrl {

using namespace builder::details;

MultibodyModelBuilderBase::MultibodyModelBuilderBase(ModelPtr model, LinkNameToIndexMapPtr link_name_to_idx)
  : model_{ std::move(model) }, link_name_to_idx_{ std::move(link_name_to_idx) }
{}

MultibodyModelBuilderBase::MultibodyModelBuilderBase(ModelPtr model,
  LinkNameToIndexMapPtr link_name_to_idx,
  LoggerPtr logger)
  : model_{ std::move(model) }, link_name_to_idx_{ std::move(link_name_to_idx) }, logger_{ std::move(logger) }
{}

MultibodyModelBuilderBase &MultibodyModelBuilderBase::set_logger(spdlog::logger logger)
{
  logger_->swap(logger);
  return *this;
}

MultibodyModelBuilderWithoutRoot::MultibodyModelBuilderWithoutRoot()
  : MultibodyModelBuilderBase{ std::make_shared<MultibodyModel>(), std::make_shared<LinkNameToIndexMap>() }
{}

MultibodyModelBuilder MultibodyModelBuilderWithoutRoot::set_root(const urdf::ModelInterfaceSharedPtr &urdf_tree)
{
  model_->name_ = urdf_tree->getName();
  ::urdf::LinkConstSharedPtr const root = urdf_tree->getRoot();
  (*link_name_to_idx_)[root->name] = -1;

  return { model_, link_name_to_idx_, logger_ };
}

MultibodyModelBuilderWithoutRoot &MultibodyModelBuilderWithoutRoot::set_logger(spdlog::logger logger)
{
  MultibodyModelBuilderBase::set_logger(std::move(logger));
  return *this;
}

MultibodyModelBuilder::MultibodyModelBuilder(ModelPtr model, LinkNameToIndexMapPtr link_name_to_idx, LoggerPtr logger)
  : MultibodyModelBuilderBase(std::move(model), std::move(link_name_to_idx), std::move(logger))
{}

MultibodyModelBuilder &MultibodyModelBuilder::set_logger(spdlog::logger logger)
{
  MultibodyModelBuilderBase::set_logger(std::move(logger));
  return *this;
}

constexpr double EPSILON = 1e-9;

bool operator==(::urdf::Vector3 const &lhs, urdf::Vector3 const &rhs)
{
  auto close_enough = [](double a, double b) { return std::abs(a - b) < EPSILON; };
  return close_enough(lhs.x, rhs.x) && close_enough(lhs.y, rhs.y) && close_enough(lhs.z, rhs.z);
}

SE3 convert_from_urdf(::urdf::Pose const &pose)
{
  ::urdf::Vector3 const &p = pose.position;
  ::urdf::Rotation const &q = pose.rotation;
  SE3 T;
  T.topLeftCorner<3, 3>() = Quaternion(q.w, q.x, q.y, q.z).matrix();
  T.topRightCorner<3, 1>() = Vector3(p.x, p.y, p.z);
  T(3, 3) = 1.0;

  return T;
}

namespace {
  std::array<double, 3> convert_to_array(::urdf::Vector3 const &vec)
  {
    return { vec.x, vec.y, vec.z };
  }
}

MultibodyModelBuilder &MultibodyModelBuilder::add_link(::urdf::LinkConstSharedPtr const &link)
{
  logger_->debug("Processing link: {}", link->name);

  if (link->parent_joint->type == ::urdf::Joint::REVOLUTE) {
    auto joint = RevoluteJoint(determine_joint_axis(convert_to_array(link->parent_joint->axis)));
    model_->joints_.emplace_back(joint);
  } else if (link->parent_joint->type == ::urdf::Joint::FIXED) {
    if (link->name == "ee_link") {
      logger_->debug("Found end effector link");
      model_->nTee_ = convert_from_urdf(link->parent_joint->parent_to_joint_origin_transform);
      // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
      logger_->debug("End effector transform: \n{}", matrix_to_str(model_->nTee_.value()));
    }
    return *this;
  } else {
    not_implemented();
  }

  std::string const &parent_link_name = link->getParent()->name;
  if (link_name_to_idx_->find(parent_link_name) == link_name_to_idx_->end()) {
    throw std::invalid_argument(fmt::format(
      "Failed to process link: {}. Parent link {} not found, it must be added before", link->name, parent_link_name));
  }
  int const parent_idx = (*link_name_to_idx_)[parent_link_name];
  model_->parent_.emplace_back(parent_idx);
  (*link_name_to_idx_)[link->name] = static_cast<int>((*model_).parent_.size() - 1);
  model_->n_bodies_++;

  SE3 T = Tinv(convert_from_urdf(link->parent_joint->parent_to_joint_origin_transform));
  auto X = Ad(T);
  logger_->debug("Spatial transform: \n{}", matrix_to_str(X));
  model_->X_tree_.emplace_back(X);

  ::urdf::Inertial const &Y = *link->inertial;
  ::urdf::Vector3 const &p = Y.origin.position;
  ::urdf::Rotation const &q = Y.origin.rotation;

  Vector3 p_C_i = Vector3(p.x, p.y, p.z);
  SO3 const &R_i_C = Quaternion(q.w, q.x, q.y, q.z).matrix();
  logger_->debug("CoM position: \n{}", matrix_to_str(p_C_i));
  logger_->debug("CoM orientation:\n{}", matrix_to_str(R_i_C));

  RotationalInertia rotI_C_C;
  rotI_C_C << Y.ixx, Y.ixy, Y.ixz, Y.ixy, Y.iyy, Y.iyz, Y.ixz, Y.iyz, Y.izz;
  logger_->debug("Rotation inertia in CoM frame:\n{}", matrix_to_str(rotI_C_C));

  RotationalInertia const rotI_C_i = R_i_C * rotI_C_C * R_i_C.transpose();
  logger_->debug("Rotation inertia in link frame:\n{}", matrix_to_str(rotI_C_i));

  auto I = I_from_rotinertia_about_com(rotI_C_i, p_C_i, Y.mass);
  logger_->debug("Spatial inertia:\n{}", matrix_to_str(I));
  model_->I_.emplace_back(I);

  return *this;
}

constexpr std::array<AxisVec, 3> const axis_mappings{ AxisVec{ { 1., 0., 0. }, JointAxis::X },
  AxisVec{ { 0., 1., 0. }, JointAxis::Y },
  AxisVec{ { 0., 0., 1. }, JointAxis::Z } };

JointAxis builder::details::determine_joint_axis(std::array<double, 3> const &vec)
{
  for (auto const &[axis, joint_axis] : axis_mappings) {
    if (vec == axis) { return joint_axis; }
  }

  return JointAxis::UNALIGNED;
}

}// namespace legged_ctrl
