#ifndef LCC_MJXML_DEFAULTS_HPP
#define LCC_MJXML_DEFAULTS_HPP

#include "legged_control_cpp/mjxml/mjxml.hpp"
#include <memory>
#include <string>
#include <tinyxml2.h>
#include <unordered_map>
#include <utility>
#include <vector>

namespace legged_ctrl::mjxml {

namespace details {

  struct Default
  {
    using DefaultPtr = std::unique_ptr<Default>;

    std::string class_name{ "main" };
    std::unordered_map<std::string, DummyModelElement> elements{};
    std::vector<DefaultPtr> children{};

    Default() = default;

    explicit Default(std::string name) : class_name(std::move(name)) {}

    // Add a child Default to the current Default
    void addChild(DefaultPtr child);

    void add_model_element(DummyModelElement model_element);
  };

  Default::DefaultPtr construct_defaults_tree(tinyxml2::XMLElement const *top_level_default);

  std::unordered_map<std::string, ModelElementClass> traverse_defaults_tree(Default::DefaultPtr const &root);
}// namespace details

}// namespace legged_ctrl::mjxml

#endif