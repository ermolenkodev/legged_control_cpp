#include "mjxml_defaults.hpp"
#include <tinyxml2.h>

namespace legged_ctrl::mjxml::details {

void Default::addChild(Default::DefaultPtr child) { children.push_back(std::move(child)); }

void Default::add_model_element(DummyModelElement model_element)
{
  elements.insert({ std::move(model_element.name), std::move(model_element) });
}

void build_node(tinyxml2::XMLElement const *default_xml, std::unique_ptr<Default> &child_node);

void construct_defaults_tree(tinyxml2::XMLElement const *default_xml, Default::DefaultPtr &node)
{
  char const* class_attribute = default_xml->Attribute("class");
  auto child_node = std::make_unique<Default>(class_attribute);
  build_node(default_xml, child_node);
  node->addChild(std::move(child_node));
}

void build_node(tinyxml2::XMLElement const *default_xml, std::unique_ptr<Default> &child_node)
{
  for (auto const *child = default_xml->FirstChildElement(); child != nullptr; child = child->NextSiblingElement()) {
    if (std::string_view(child->Name()) == "default") {
      construct_defaults_tree(child, child_node);
    } else {
      DummyModelElement model_element(child->Name());
      model_element.set_attributes_from(child);
      child_node->add_model_element(std::move(model_element));
    }
  }
}

Default::DefaultPtr construct_defaults_tree(tinyxml2::XMLElement const *top_level_default)
{
  const char *class_attribute = top_level_default->Attribute("class");
  auto root = class_attribute != nullptr ? std::make_unique<Default>(class_attribute) : std::make_unique<Default>();
  build_node(top_level_default, root);

  return root;
}

void traverse_defaults_tree(Default::DefaultPtr const &node,
  std::unordered_map<std::string, ModelElementClass> &model_elements)
{
  for (auto const &[tag, element] : node->elements) {
    model_elements[node->class_name].upsert(tag, element);
  }

  for (auto const &child : node->children) {
    model_elements[child->class_name] = model_elements[node->class_name];
    traverse_defaults_tree(child, model_elements);
  }
}

}// namespace legged_ctrl::mjxml::details
