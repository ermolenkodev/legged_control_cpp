#include <catch2/catch_test_macros.hpp>

#include "legged_control_cpp/inverse_dynamics.hpp"
#include "legged_control_cpp/model.hpp"
#include "test_data/inverse_dynamics_test_data.hpp"
#include "test_data/bundled_models.hpp"

using namespace legged_ctrl;

TEST_CASE("Inverse dynamics UR5", "[inverse_dynamics]")
{
  MultibodyModel const model = ur5_model();
  for (auto const &[cfg, ext, expected_tau] : ur5_rnea_test_configurations()) {
    auto const tau = rnea(model, cfg, ext);
    REQUIRE(tau.isApprox(expected_tau, PRECISION));
  }
}

TEST_CASE("Inverse dynamics IIWA14", "[inverse_dynamics]")
{
  MultibodyModel const model = iiwa14_model_urdf();
  for (auto const &[cfg, ext, expected_tau] : iiwa_rnea_test_configurations()) {
    auto const tau = rnea(model, cfg, ext);
    REQUIRE(tau.isApprox(expected_tau, PRECISION));
  }
}

TEST_CASE("Inverse dynamics IIWA14 mjxml", "[inverse_dynamics]")
{
  MultibodyModel const model = iiwa14_model_mjxml();
  for (auto const &[cfg, ext, expected_tau] : iiwa_rnea_test_configurations()) {
    auto const tau = rnea(model, cfg, ext);
    REQUIRE(tau.isApprox(expected_tau, PRECISION));
  }
}
