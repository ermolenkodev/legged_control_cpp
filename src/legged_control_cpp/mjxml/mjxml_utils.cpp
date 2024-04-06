#include "mjxml_utils.hpp"

namespace legged_ctrl::mjxml::details {

Vector3 parse_vector3(const std::string &pos_attribute)
{
  double x, y, z;
  std::istringstream ss(pos_attribute);

  if (!(ss >> x >> y >> z)) {
    throw std::invalid_argument(fmt::format("Invalid format for Vector3. Received: {}", pos_attribute));
  }

  return { x, y, z };
}

Quaternion parse_quaternion(const std::string &quat_attribute)
{
  double w, x, y, z;
  std::istringstream ss(quat_attribute);

  if (!(ss >> w >> x >> y >> z)) {
    throw std::invalid_argument(fmt::format("Invalid format for quaternion. Received: {}", quat_attribute));
  }

  return Quaternion{ w, x, y, z }.normalized();
}

std::string
  get_attribute(tinyxml2::XMLElement const *xml, std::string const &attribute_name, std::string const &default_value)
{
  if (xml->Attribute(attribute_name.c_str()) == nullptr) { return default_value; }
  return xml->Attribute(attribute_name.c_str());
}

std::string get_attribute(tinyxml2::XMLElement const *xml,
  std::string const &attribute_name,
  std::unordered_map<std::string, std::string> const &class_defaults,
  std::string const &default_value)
{
  if (xml->Attribute(attribute_name.c_str()) != nullptr) { return xml->Attribute(attribute_name.c_str()); }

  if (class_defaults.find(attribute_name) != class_defaults.end()) { return class_defaults.at(attribute_name); }

  return default_value;
}

std::array<double, 3> convert_to_array(Vector3 const &vec)
{
  return { vec[0], vec[1], vec[2] };
}

}// namespace legged_ctrl::mjxml::details
