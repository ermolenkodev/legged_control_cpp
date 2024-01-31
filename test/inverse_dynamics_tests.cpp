#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "legged_control_cpp/inverse_dynamics.hpp"
#include "test_data/inverse_dynamics_test_data.hpp"

using namespace legged_ctrl;

TEST_CASE("Inverse dynamics UR5", "[inverse_dynamics]")
{
  MultibodyModel const model = ur5_model();
  for (auto const &[cfg, ext, expected_tau] : ur5_rnea_test_configurations()) {
    auto const tau = rnea(model, cfg, ext);
    REQUIRE(tau.isApprox(expected_tau, PRECISION));
  }
}
