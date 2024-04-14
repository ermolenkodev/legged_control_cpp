#include "mjxml_utils.hpp"

namespace legged_ctrl::mjxml::details {

Vector3 parse_vector3(std::string const &pos_attribute)
{
  double x, y, z;// NOLINT
  std::istringstream ss(pos_attribute);

  if (!(ss >> x >> y >> z)) {
    throw std::invalid_argument(fmt::format("Invalid format for Vector3. Received: {}", pos_attribute));
  }

  return { x, y, z };
}

Quaternion parse_quaternion(std::string const &quat_attribute)
{
  double w, x, y, z;// NOLINT
  std::istringstream ss(quat_attribute);

  if (!(ss >> w >> x >> y >> z)) {
    throw std::invalid_argument(fmt::format("Invalid format for quaternion. Received: {}", quat_attribute));
  }

  return Quaternion{ w, x, y, z }.normalized();
}

std::string get_attribute(tinyxml2::XMLElement const *xml,
  AttributeName const &attribute_name,
  AttributeValue const &default_value)
{
  if (xml->Attribute(attribute_name.value.c_str()) == nullptr) { return default_value.value; }
  return xml->Attribute(attribute_name.value.c_str());
}

std::string get_attribute(tinyxml2::XMLElement const *xml,
  AttributeName const &attribute_name,
  std::unordered_map<std::string, std::string> const &class_defaults,
  AttributeValue const &default_value)
{
  if (xml->Attribute(attribute_name.value.c_str()) != nullptr) { return xml->Attribute(attribute_name.value.c_str()); }

  if (class_defaults.find(attribute_name.value) != class_defaults.end()) {
    return class_defaults.at(attribute_name.value);
  }

  return default_value.value;
}

std::array<double, 3> convert_to_array(Vector3 const &vec) { return { vec[0], vec[1], vec[2] }; }

}// namespace legged_ctrl::mjxml::details
