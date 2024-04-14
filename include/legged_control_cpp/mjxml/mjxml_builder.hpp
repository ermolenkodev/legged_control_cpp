#ifndef LC_CPP_MJXML_BUILDER_HPP
#define LC_CPP_MJXML_BUILDER_HPP

#include "legged_control_cpp/builder.hpp"
#include "mjxml.hpp"

namespace legged_ctrl::mjxml {

class MjxmlMultibodyModelBuilder;

class MjxmlModelBuilderWithoutRoot : public MultibodyModelBuilderBase
{
public:
  MjxmlModelBuilderWithoutRoot();
  MjxmlMultibodyModelBuilder set_root(tinyxml2::XMLElement const *root_xml);
  MjxmlModelBuilderWithoutRoot &set_logger(spdlog::logger logger) override;
};

class MjxmlMultibodyModelBuilder : public MultibodyModelBuilderBase
{
public:
  MjxmlMultibodyModelBuilder &add_link(tinyxml2::XMLElement const *link_xml,
    tinyxml2::XMLElement const *parent_link_xml,
    std::string const &inherited_class);

  MjxmlMultibodyModelBuilder &set_logger(spdlog::logger logger) override;
  MjxmlMultibodyModelBuilder &set_defaults(tinyxml2::XMLElement const *defaults_xml);
private:
  std::unordered_map<std::string, ModelElementClass> model_elements_classes;

  MjxmlMultibodyModelBuilder(ModelPtr model, LinkNameToIndexMapPtr link_name_to_idx, LoggerPtr logger);

  std::unordered_map<std::string, std::string> get_attributes(std::string const &model_element,
    std::string const &class_name);

  SpatialMatrix process_inertia(const std::string &inherited_class, const tinyxml2::XMLElement *inertial);
  RevoluteJoint process_joint(const std::string &inherited_class, const tinyxml2::XMLElement *joint_xml);

  friend class MjxmlModelBuilderWithoutRoot;
};

}// namespace legged_ctrl::mjxml

#endif
