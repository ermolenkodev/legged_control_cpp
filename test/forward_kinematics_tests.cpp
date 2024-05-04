#include <catch2/catch_test_macros.hpp>

#include "legged_control_cpp/forward_kinematics.hpp"
#include "legged_control_cpp/model.hpp"
#include "test_data/bundled_models.hpp"
#include "test_data/forward_kinematics_test_data.hpp"

using namespace legged_ctrl;

TEST_CASE("Spatial velocity IIWA4", "[forward_kinematics]")
{
  MultibodyModel const model = iiwa14_model_urdf();
  for (auto const &[cfg, expected_V] : iiwa14_spatial_vel_test_configurations()) {
    auto [V, _] = legged_ctrl::compute_end_effector_forward_kinematics(model, cfg, ReferenceFrame::LOCAL_WORLD_ALIGNED);
    REQUIRE(V.isApprox(expected_V, PRECISION));
  }
}

TEST_CASE("Spatial acceleration IIWA4", "[forward_kinematics]")
{
  MultibodyModel const model = iiwa14_model_urdf();
  for (auto const &[cfg, expected_A] : iiwa14_spatial_acc_test_configurations()) {
    auto [_, A] = legged_ctrl::compute_end_effector_forward_kinematics(model, cfg, ReferenceFrame::LOCAL_WORLD_ALIGNED);
    REQUIRE(A.isApprox(expected_A, PRECISION));
  }
}

TEST_CASE("Classical acceleration IIWA4", "[forward_kinematics]")
{
  MultibodyModel const model = iiwa14_model_urdf();
  for (auto const &[cfg, expected_a] : iiwa14_spatial_classical_acc_test_configurations()) {
    auto a = legged_ctrl::compute_end_effector_classical_acceleration(model, cfg, ReferenceFrame::LOCAL_WORLD_ALIGNED);
    REQUIRE(a.isApprox(expected_a, PRECISION));
  }
}
