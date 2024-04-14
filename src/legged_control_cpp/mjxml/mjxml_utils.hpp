#ifndef LCC_MJXML_PARSING_HPP
#define LCC_MJXML_PARSING_HPP

#include "legged_control_cpp/type_aliases.hpp"
#include "legged_control_cpp/utilities.hpp"
#include <tinyxml2.h>

namespace legged_ctrl::mjxml::attributes {
using AttributeName = Tagged<struct AttributeNameTag, std::string>;
using AttributeValue = Tagged<struct AttributeValueTag, std::string>;
}// namespace legged_ctrl::mjxml::attributes

namespace legged_ctrl::mjxml::details {

using namespace legged_ctrl::mjxml::attributes;

Vector3 parse_vector3(std::string const &pos_attribute);

Quaternion parse_quaternion(std::string const &quat_attribute);

std::string get_attribute(tinyxml2::XMLElement const *xml,
  AttributeName const &attribute_name,
  AttributeValue const &default_value);

std::string get_attribute(tinyxml2::XMLElement const *xml,
  AttributeName const &attribute_name,
  std::unordered_map<std::string, std::string> const &class_defaults,
  AttributeValue const &default_value);

std::array<double, 3> convert_to_array(Vector3 const &vec);

}// namespace legged_ctrl::mjxml::details

namespace legged_ctrl::mjxml::attributes {

const AttributeName POS{ "pos" };
const AttributeName QUAT{ "quat" };
const AttributeName CLASS{ "class" };
const AttributeName BODY{ "body" };
const AttributeName TYPE{ "type" };
const AttributeName AXIS{ "axis" };
const AttributeName DIAGINERTIA{ "diaginertia" };
const AttributeName MASS{ "mass" };
const AttributeName CHILDCLASS{ "childclass" };

const AttributeValue DEFAULT_POS{ "0 0 0" };
const AttributeValue DEFAULT_QUAT{ "1 0 0 0" };
const AttributeValue DEFAULT_JOINT{ "hinge" };
const AttributeValue DEFAULT_AXIS{ "0 0 1" };
const AttributeValue DEFAULT_DIAGINERTIA{ "0 0 0" };
const AttributeValue ZERO{ "0" };

}// namespace legged_ctrl::mjxml::attributes

#endif// LCC_MJXML_PARSING_HPP
