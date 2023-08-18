#include "legged_control_cpp/inverse_dynamics.hpp"
#include "legged_control_cpp/model.hpp"
#include "legged_control_cpp/urdf.hpp"
#include <spdlog/sinks/stdout_color_sinks.h>

using namespace legged_ctrl;

int main(int argc, char **argv)
{
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  std::string const urdf_path = argc < 2 ? "assets/ur5.urdf" : argv[1];

  auto logger = *spdlog::stdout_color_mt("lcc_urdf_parser");
  logger.set_level(spdlog::level::debug);

  MultibodyModel const model = legged_ctrl::urdf_parser::parse_urdf(urdf_path, logger);
  auto ph = PhysicsDescription{ model };
  auto config = SystemConfiguration{ VectorX::Zero(model.num_bodies()) };
  auto end_effector_force = SpatialVector::Ones();
  auto ext = ExternalForces{ end_effector_force };

  std::cout << "Tau:\n" << legged_ctrl::rnea(ph, config, ext) << '\n';

  return 0;
}
