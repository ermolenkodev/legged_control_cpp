#include "legged_control_cpp/builder.hpp"

#include "fmt/core.h"
#include "legged_control_cpp/type_aliases.hpp"
#include "legged_control_cpp/utilities.hpp"
#include <utility>


namespace legged_ctrl {

using namespace builder::details;

MultibodyModelBuilderBase::MultibodyModelBuilderBase(ModelPtr model_, LinkNameToIndexMapPtr link_name_to_idx_)
  : model{ std::move(model_) }, link_name_to_idx{ std::move(link_name_to_idx_) }
{}

MultibodyModelBuilderBase::MultibodyModelBuilderBase(ModelPtr model_,
  LinkNameToIndexMapPtr link_name_to_idx_,
  LoggerPtr logger_)
  : model{ std::move(model_) }, link_name_to_idx{ std::move(link_name_to_idx_) }, logger{ std::move(logger_) }
{}

MultibodyModelBuilderBase &MultibodyModelBuilderBase::set_logger(spdlog::logger logger_)
{
  this->logger->swap(logger_);
  return *this;
}

MultibodyModelBuilderWithoutRoot::MultibodyModelBuilderWithoutRoot()
  : MultibodyModelBuilderBase{ std::make_shared<MultibodyModel>(), std::make_shared<LinkNameToIndexMap>() }
{}

MultibodyModelBuilder MultibodyModelBuilderWithoutRoot::set_root(const urdf::ModelInterfaceSharedPtr &urdf_tree)
{
  model->name = urdf_tree->getName();
  ::urdf::LinkConstSharedPtr const root = urdf_tree->getRoot();
  (*link_name_to_idx)[root->name] = -1;

  return { model, link_name_to_idx, logger };
}

MultibodyModelBuilderWithoutRoot &MultibodyModelBuilderWithoutRoot::set_logger(spdlog::logger logger_)
{
  MultibodyModelBuilderBase::set_logger(std::move(logger_));
  return *this;
}

MultibodyModelBuilder::MultibodyModelBuilder(ModelPtr model_,
  LinkNameToIndexMapPtr link_name_to_idx_,
  LoggerPtr logger_)
  : MultibodyModelBuilderBase(std::move(model_), std::move(link_name_to_idx_), std::move(logger_))
{}

MultibodyModelBuilder &MultibodyModelBuilder::set_logger(spdlog::logger logger_)
{
  MultibodyModelBuilderBase::set_logger(std::move(logger_));
  return *this;
}

constexpr double EPSILON = 1e-9;

bool operator==(::urdf::Vector3 const &lhs, urdf::Vector3 const &rhs)
{
  auto close_enough = [](double a, double b) { return std::abs(a - b) < EPSILON; };
  return close_enough(lhs.x, rhs.x) && close_enough(lhs.y, rhs.y) && close_enough(lhs.z, rhs.z);
}

SE3 urdf_pose_to_SE3(::urdf::Pose const &pose)
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
  std::array<double, 3> convert_to_array(::urdf::Vector3 const &vec) { return { vec.x, vec.y, vec.z }; }

  JointAxis map_to_model_axis(::urdf::Vector3 const &v) { return determine_joint_axis(convert_to_array(v)); }

  void add_joint_to_model(MultibodyModel &model, ::urdf::JointConstSharedPtr const &joint)
  {
    if (joint->type == ::urdf::Joint::REVOLUTE) {
      auto revolute_joint = RevoluteJoint(map_to_model_axis(joint->axis));
      model.joints.emplace_back(revolute_joint);
    } else if (joint->type == ::urdf::Joint::FIXED) {
      // noop
    } else {
      not_implemented();
    }
  }

  bool is_end_effector(::urdf::LinkConstSharedPtr const &link)
  {
    return link->name == "ee_link";// dummy implementation
  }

  std::pair<Vector3, SO3> get_com_position_and_orientation(::urdf::Inertial const &Y)
  {
    ::urdf::Vector3 const &p = Y.origin.position;
    ::urdf::Rotation const &q = Y.origin.rotation;

    Vector3 p_C_i = Vector3(p.x, p.y, p.z);
    SO3 const R_i_C = Quaternion(q.w, q.x, q.y, q.z).matrix();

    return { p_C_i, R_i_C };
  }
}// namespace

int MultibodyModelBuilder::get_parent_idx(::urdf::LinkConstSharedPtr const &link) const
{
  std::string const &parent_link_name = link->getParent()->name;
  if (link_name_to_idx->find(parent_link_name) == link_name_to_idx->end()) {
    throw std::invalid_argument(fmt::format(
      "Failed to process link: {}. Parent link {} not found, it must be added before", link->name, parent_link_name));
  }

  return link_name_to_idx->at(parent_link_name);
}

MultibodyModelBuilder &MultibodyModelBuilder::add_link_and_joint_to_model(::urdf::LinkConstSharedPtr const &link,
  ::urdf::JointConstSharedPtr const &joint)
{
  logger->debug("Processing link: {}", link->name);

  if (joint->type == ::urdf::Joint::FIXED and not is_end_effector(link)) { return *this; }

  if (is_end_effector(link)) {
    logger->debug("Found end effector link");
    model->nTee = urdf_pose_to_SE3(link->parent_joint->parent_to_joint_origin_transform);
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
    logger->debug("End effector transform: \n{}", matrix_to_str(model->nTee.value()));

    return *this;
  }

  add_joint_to_model(*model, joint);

  int const parent_idx = get_parent_idx(link);
  model->parent.emplace_back(parent_idx);
  (*link_name_to_idx)[link->name] = model->n_bodies++;

  SE3 T = Tinv(urdf_pose_to_SE3(joint->parent_to_joint_origin_transform));
  auto X = Ad(T);
  logger->debug("Spatial transform: \n{}", matrix_to_str(X));
  model->X_tree.emplace_back(X);

  ::urdf::Inertial const &Y = *link->inertial;
  auto [ipC, iRC] = get_com_position_and_orientation(Y);
  logger->debug("CoM position: \n{}", matrix_to_str(ipC));
  logger->debug("CoM orientation:\n{}", matrix_to_str(iRC));

  RotationalInertia C_rotI_C;
  C_rotI_C << Y.ixx, Y.ixy, Y.ixz, Y.ixy, Y.iyy, Y.iyz, Y.ixz, Y.iyz, Y.izz;
  logger->debug("Rotation inertia in CoM frame:\n{}", matrix_to_str(C_rotI_C));

  RotationalInertia const i_rotI_C = iRC * C_rotI_C * iRC.transpose();
  logger->debug("Rotation inertia in link frame:\n{}", matrix_to_str(i_rotI_C));

  auto I = apply_steiners_theorem(i_rotI_C, ipC, Y.mass);
  logger->debug("Spatial inertia:\n{}", matrix_to_str(I));
  model->I.emplace_back(I);

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
