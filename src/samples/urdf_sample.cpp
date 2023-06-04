#include "legged_control_cpp/model.hpp"
#include "legged_control_cpp/urdf.hpp"

using namespace legged_ctrl;

int main(int argc, char **argv)
{
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  std::string const urdf_path = argc < 2 ? "assets/ur4.urdf" : argv[1];
  MultibodyModel const model = legged_ctrl::urdf_parser::parse_urdf(urdf_path);
  std::cout << model << std::endl;
  return 0;
}