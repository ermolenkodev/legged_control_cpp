#include "legged_control_cpp/mjxml/mjxml_builder.hpp"

#include "legged_control_cpp/utilities.hpp"
#include "mjxml_defaults.hpp"
#include "mjxml_utils.hpp"
#include <utility>

namespace legged_ctrl::mjxml {

using namespace details;

MjxmlMultibodyModelBuilder MjxmlModelBuilderWithoutRoot::set_root(tinyxml2::XMLElement const *root_xml)
{
  (*link_name_to_idx)[root_xml->Attribute("name")] = -1;
  return { model, link_name_to_idx, logger };
}

MjxmlModelBuilderWithoutRoot::MjxmlModelBuilderWithoutRoot()
  : MultibodyModelBuilderBase{ std::make_shared<MultibodyModel>(), std::make_shared<LinkNameToIndexMap>() }
{}

MjxmlModelBuilderWithoutRoot &MjxmlModelBuilderWithoutRoot::set_logger(spdlog::logger logger_)
{
  MultibodyModelBuilderBase::set_logger(std::move(logger_));
  return *this;
}

// This function makes unnecessary copy of attributes map, but it is not a problem for now
std::unordered_map<std::string, std::string>
  MjxmlMultibodyModelBuilder::get_attributes(std::string const &model_element, std::string const &class_name)
{
  if (model_elements_classes.find(class_name) == model_elements_classes.end()) { return {}; }
  if (!model_elements_classes[class_name].contains(model_element)) { return {}; }

  return model_elements_classes[class_name].at(model_element).attributes;
}

SE3 get_link_transform(tinyxml2::XMLElement const *link_xml,
  std::unordered_map<std::string, std::string> const &link_defaults)
{
  Vector3 const parent_P_i = parse_vector3(get_attribute(link_xml, POS, link_defaults, DEFAULT_POS));
  Quaternion const q = parse_quaternion(get_attribute(link_xml, QUAT, link_defaults, DEFAULT_QUAT));
  SO3 const parent_R_i = q.matrix();

  SE3 T;
  T.topLeftCorner<3, 3>() = parent_R_i;
  T.topRightCorner<3, 1>() = parent_P_i;
  T(3, 3) = 1.0;

  return T;
}

MjxmlMultibodyModelBuilder &MjxmlMultibodyModelBuilder::add_link(tinyxml2::XMLElement const *link_xml,
  tinyxml2::XMLElement const *parent_link_xml,
  std::string const &inherited_class)
{
  std::string const link_name = link_xml->Attribute("name");
  logger->debug("Processing link: {}", link_name);

  tinyxml2::XMLElement const *inertial_xml = link_xml->FirstChildElement("inertial");
  if (inertial_xml != nullptr) {
    model->I.emplace_back(process_inertia(inherited_class, inertial_xml));
  } else {
    std::string const link_class = get_attribute(link_xml, CLASS, AttributeValue{ inherited_class });
    model->nTee = get_link_transform(link_xml, get_attributes("body", link_class));

    return *this;
  }

  std::string const parent_link_name = parent_link_xml->Attribute("name");
  if (link_name_to_idx->find(parent_link_name) == link_name_to_idx->end()) {
    throw std::invalid_argument(fmt::format(
      "Failed to process link: {}. Parent link {} not found, it must be added before", link_name, parent_link_name));
  }

  int const parent_idx = (*link_name_to_idx)[parent_link_name];
  model->parent.emplace_back(parent_idx);
  (*link_name_to_idx)[link_name] = static_cast<int>((*model).parent.size() - 1);
  model->n_bodies++;

  std::string const link_class = get_attribute(link_xml, CLASS, AttributeValue{ inherited_class });
  SE3 const T = Tinv(get_link_transform(link_xml, get_attributes("body", link_class)));
  SpatialMatrix const X = Ad(T);
  model->X_tree.emplace_back(X);

  tinyxml2::XMLElement const *joint_xml = link_xml->FirstChildElement("joint");
  if (joint_xml != nullptr) {
    RevoluteJoint joint = process_joint(inherited_class, joint_xml);
    model->joints.emplace_back(joint);
  }

  return *this;
}

RevoluteJoint MjxmlMultibodyModelBuilder::process_joint(const std::string &inherited_class,
  const tinyxml2::XMLElement *joint_xml)
{
  std::string const class_attribute = get_attribute(joint_xml, CLASS, AttributeValue{ inherited_class });
  std::unordered_map<std::string, std::string> const defaults = get_attributes("joint", class_attribute);
  std::string const type_attribute = get_attribute(joint_xml, TYPE, defaults, DEFAULT_JOINT);

  Vector3 const axis = parse_vector3(get_attribute(joint_xml, AXIS, defaults, DEFAULT_AXIS));

  if (type_attribute != "hinge") { not_implemented(); }

  auto joint = RevoluteJoint(builder::details::determine_joint_axis(convert_to_array(axis)));

  return joint;
}

SpatialMatrix MjxmlMultibodyModelBuilder::process_inertia(const std::string &inherited_class,
  const tinyxml2::XMLElement *inertial)
{
  std::string const inertial_class = get_attribute(inertial, CLASS, AttributeValue{ inherited_class });
  std::unordered_map<std::string, std::string> const defaults = get_attributes("inertial", inertial_class);

  Vector3 const i_P_C = parse_vector3(get_attribute(inertial, POS, defaults, DEFAULT_POS));
  SO3 const i_R_C = parse_quaternion(get_attribute(inertial, QUAT, defaults, DEFAULT_QUAT)).matrix();
  logger->debug("CoM position: \n{}", matrix_to_str(i_P_C));
  logger->debug("CoM orientation:\n{}", matrix_to_str(i_R_C));

  Vector3 const diag_inertia = parse_vector3(get_attribute(inertial, DIAGINERTIA, defaults, DEFAULT_DIAGINERTIA));
  RotationalInertia rotI_C_C;
  rotI_C_C << diag_inertia(0), 0, 0, 0, diag_inertia(1), 0, 0, 0, diag_inertia(2);
  logger->debug("Rotation inertia in CoM frame:\n{}", matrix_to_str(rotI_C_C));

  RotationalInertia const rotI_C_i = i_R_C * rotI_C_C * i_R_C.transpose();
  logger->debug("Rotation inertia in link frame:\n{}", matrix_to_str(rotI_C_i));

  double const mass = std::stod(get_attribute(inertial, MASS, defaults, ZERO));
  auto I = I_from_rotinertia_about_com(rotI_C_i, i_P_C, mass);
  logger->debug("Spatial inertia:\n{}", matrix_to_str(I));

  return I;
}

MjxmlMultibodyModelBuilder::MjxmlMultibodyModelBuilder(ModelPtr model_,
  LinkNameToIndexMapPtr link_name_to_idx_,
  LoggerPtr logger_)
  : MultibodyModelBuilderBase(std::move(model_), std::move(link_name_to_idx_), std::move(logger_))
{}

MjxmlMultibodyModelBuilder &MjxmlMultibodyModelBuilder::set_logger(spdlog::logger logger_)
{
  this->logger->swap(logger_);
  return *this;
}

MjxmlMultibodyModelBuilder &MjxmlMultibodyModelBuilder::set_defaults(const tinyxml2::XMLElement *defaults_xml)
{
  if (defaults_xml == nullptr) { return *this; }
  Default::DefaultPtr const defaults_tree = construct_defaults_tree(defaults_xml);
  model_elements_classes = traverse_defaults_tree(defaults_tree);

  return *this;
}

}// namespace legged_ctrl::mjxml
