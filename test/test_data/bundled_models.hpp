#ifndef LCC_BUNDLED_MODELS_HPP
#define LCC_BUNDLED_MODELS_HPP

#include "legged_control_cpp/mjxml/mjxml.hpp"
#include "legged_control_cpp/type_aliases.hpp"
#include "legged_control_cpp/urdf.hpp"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <string>
#include <filesystem>

using namespace legged_ctrl;

std::string get_assets_path(const char* filePath) {
  std::filesystem::path path = filePath;
  path = path.parent_path().parent_path().parent_path(); // Move up three levels
  path /= "assets";
  return path.string();
}


#define ASSETS_PATH get_assets_path(__FILE__)

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

#endif
