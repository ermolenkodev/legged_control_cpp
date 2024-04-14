#include "mjxml_defaults.hpp"
#include <tinyxml2.h>
#include <stack>

namespace legged_ctrl::mjxml::details {

void Default::addChild(Default::DefaultPtr child) { children.push_back(std::move(child)); }

void Default::add_model_element(DummyModelElement model_element)
{
  elements.insert({ std::move(model_element.name), std::move(model_element) });
}

Default::DefaultPtr construct_defaults_tree(tinyxml2::XMLElement const *top_level_default)
{
  const char *class_attribute = top_level_default->Attribute("class");
  auto root = class_attribute != nullptr ? std::make_unique<Default>(class_attribute) : std::make_unique<Default>();

  std::stack<std::pair<tinyxml2::XMLElement const *, Default *>> stk;
  stk.emplace(top_level_default, root.get());

  while (!stk.empty()) {
    auto [current_xml, current_node] = stk.top();
    stk.pop();

    class_attribute = current_xml->Attribute("class");
    auto child_node = class_attribute != nullptr ? std::make_unique<Default>(class_attribute) : std::make_unique<Default>();

    for (auto const *child = current_xml->FirstChildElement(); child != nullptr; child = child->NextSiblingElement()) {
      if (std::string_view(child->Name()) == "default") {
        stk.emplace(child, child_node.get());
      } else {
        DummyModelElement model_element(child->Name());
        model_element.set_attributes_from(child);
        child_node->add_model_element(std::move(model_element));
      }
    }

    current_node->addChild(std::move(child_node));
  }

  return root;
}

std::unordered_map<std::string, ModelElementClass> traverse_defaults_tree(Default::DefaultPtr const &root)
{
  std::stack<Default const *> stk;
  stk.push(root.get());
  std::unordered_map<std::string, ModelElementClass> model_elements;

  while (!stk.empty()) {
    Default const *node = stk.top();
    stk.pop();

    for (auto const &[tag, element] : node->elements) {
      model_elements[node->class_name].upsert(tag, element);
    }

    for (auto const &child : node->children) {
      model_elements[child->class_name] = model_elements[node->class_name];
      stk.push(child.get());
    }
  }

  return model_elements;
}

}// namespace legged_ctrl::mjxml::details
