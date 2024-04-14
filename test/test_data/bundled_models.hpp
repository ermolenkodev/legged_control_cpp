#include "legged_control_cpp/mjxml/mjxml.hpp"
#include "legged_control_cpp/type_aliases.hpp"
#include "legged_control_cpp/urdf.hpp"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <string>

using namespace legged_ctrl;

#define ASSETS_PATH CMAKE_ASSETS_PATH

MultibodyModel ur5_model()
{
  std::string const urdf_path = std::string(ASSETS_PATH) + "/ur5.urdf";
  return urdf_parser::parse_urdf(urdf_path);
}

MultibodyModel iiwa14_model_urdf()
{
  std::string const urdf_path = std::string(ASSETS_PATH) + "/iiwa14.urdf";
  return urdf_parser::parse_urdf(urdf_path);
}

MultibodyModel iiwa14_model_mjxml()
{
  std::string const mjxml_path = std::string(ASSETS_PATH) + "/scene/iiwa14.xml";
  return mjxml::parse_mujoco_xml(mjxml_path);
}
