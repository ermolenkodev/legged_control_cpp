#include "legged_control_cpp/mjxml/mjxml.hpp"
#include "legged_control_cpp/builder.hpp"
#include "legged_control_cpp/mjxml/mjxml_builder.hpp"
#include "mjxml_utils.hpp"
#include <queue>

namespace legged_ctrl::mjxml {

using namespace legged_ctrl::mjxml::attributes;

void process_all_links(MjxmlMultibodyModelBuilder &builder, tinyxml2::XMLElement const *root_link);

MultibodyModel parse_mujoco_xml(std::string const &filename, std::optional<spdlog::logger> const &logger_opt)
{
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLError const result = doc.LoadFile(filename.c_str());

  spdlog::logger logger = logger_opt ? logger_opt.value() : *null_logger();

  if (result != tinyxml2::XML_SUCCESS) {
    logger.error("Error loading XML file!");
    throw std::runtime_error("Error loading XML file!");
  }

  tinyxml2::XMLElement const *worldbody = doc.FirstChildElement("mujoco")->FirstChildElement("worldbody");
  if (worldbody == nullptr) {
    logger.error("No worldbody found in the XML file");
    throw std::runtime_error("No worldbody found in the XML file");
  }

  tinyxml2::XMLElement const *root_link = worldbody->FirstChildElement("body");
  tinyxml2::XMLElement const *defaults_xml = doc.FirstChildElement("mujoco")->FirstChildElement("default");

  auto builder = MultibodyModel::create_from_mjxml().set_root(root_link).set_logger(logger).set_defaults(defaults_xml);
  process_all_links(builder, root_link);

  return builder.build();
}

void process_all_links(MjxmlMultibodyModelBuilder &builder, tinyxml2::XMLElement const *root_link)
{
  std::string const childclass = details::get_attribute(root_link, CHILDCLASS, AttributeValue{ "main" });

  using LinkParentClass = std::tuple<tinyxml2::XMLElement const *, tinyxml2::XMLElement const *, std::string>;
  std::queue<LinkParentClass> link_queue;
  for (auto const *link = root_link->FirstChildElement("body"); link != nullptr;
       link = link->NextSiblingElement("body")) {
    link_queue.emplace(link, root_link, childclass);
  }

  while (!link_queue.empty()) {
    auto const [link, parent, inherited_class] = link_queue.front();
    link_queue.pop();

    for (auto const *child = link->FirstChildElement("body"); child != nullptr;
         child = child->NextSiblingElement("body")) {
      link_queue.emplace(child, link, details::get_attribute(link, CHILDCLASS, AttributeValue{ inherited_class }));
    }

    builder.add_link(link, parent, inherited_class);
  }
}

DummyModelElement DummyModelElement::merge(DummyModelElement const &other) const
{
  DummyModelElement merged_element = *this;
  for (auto const &[key, value] : other.attributes) {
    if (merged_element.attributes.find(key) != merged_element.attributes.end()) {
      merged_element.attributes.at(key) = value;
    } else {
      merged_element.attributes.insert({ key, value });
    }
  }

  return merged_element;
}

void DummyModelElement::set_attributes_from(tinyxml2::XMLElement const *xml_element)
{
  for (auto const *attribute = xml_element->FirstAttribute(); attribute != nullptr; attribute = attribute->Next()) {
    attributes[attribute->Name()] = attribute->Value();
  }
}

bool ModelElementClass::contains(std::string const &element_name)
{
  return elements.find(element_name) != elements.end();
}

DummyModelElement &ModelElementClass::at(std::string const &element_name) { return elements.at(element_name); }

void ModelElementClass::insert(std::string const &element_name, DummyModelElement const &element)
{
  elements.insert({ element_name, element });
}

void ModelElementClass::upsert(std::string const &element_name, DummyModelElement const &element)
{
  if (contains(element_name)) {
    at(element_name) = at(element_name).merge(element);
  } else {
    insert(element_name, element);
  }
}

}// namespace legged_ctrl::mjxml