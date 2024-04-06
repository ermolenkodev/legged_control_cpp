#ifndef LCC_MJXML_PARSING_HPP
#define LCC_MJXML_PARSING_HPP

#include "legged_control_cpp/type_aliases.hpp"
#include <tinyxml2.h>

namespace legged_ctrl::mjxml::details {

Vector3 parse_vector3(std::string const &pos_attribute);

Quaternion parse_quaternion(std::string const &quat_attribute);

std::string
  get_attribute(tinyxml2::XMLElement const *xml, std::string const &attribute_name, std::string const &default_value);

std::string get_attribute(tinyxml2::XMLElement const *xml,
  std::string const &attribute_name,
  std::unordered_map<std::string, std::string> const &class_defaults,
  std::string const &default_value);

std::array<double, 3> convert_to_array(Vector3 const &vec);

}// namespace legged_ctrl::mjxml::details

#endif// LCC_MJXML_PARSING_HPP
