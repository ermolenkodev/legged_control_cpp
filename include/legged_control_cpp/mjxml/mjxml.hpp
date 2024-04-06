#ifndef LEGGED_CONTROL_CPP_MJXML_HPP
#define LEGGED_CONTROL_CPP_MJXML_HPP

#include "legged_control_cpp/builder.hpp"
#include "legged_control_cpp/logging.hpp"
#include "legged_control_cpp/model.hpp"
#include <tinyxml2.h>

namespace legged_ctrl::mjxml {

MultibodyModel parse_mujoco_xml(std::string const &filename, std::optional<spdlog::logger> const &logger = std::nullopt);

struct DummyModelElement
{
  std::string name{};
  std::unordered_map<std::string, std::string> attributes{};

  explicit DummyModelElement(std::string name) : name(std::move(name)) {}

  DummyModelElement merge(DummyModelElement const &other) const;

  void set_attributes_from(tinyxml2::XMLElement const *xml_element);
};

class ModelElementClass
{
public:
  bool contains(std::string const &element_name);

  DummyModelElement& at(std::string const &element_name);

  void insert(std::string const &element_name, DummyModelElement const &element);

  void upsert(std::string const &element_name, DummyModelElement const &element);
private:
  std::unordered_map<std::string, DummyModelElement> elements{};
};

}// namespace legged_ctrl::mjxml

#endif// LEGGED_CONTROL_CPP_MJXML_HPP
