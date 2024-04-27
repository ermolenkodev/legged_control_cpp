#include "legged_control_cpp/model.hpp"
#include "test_data/bundled_models.hpp"
#include <catch2/catch_test_macros.hpp>

TEST_CASE("IIWA14 mjxml parsing", "[model_parsing]")
{
  MultibodyModel const model = iiwa14_model_mjxml();
  MultibodyModel const model_urdf = iiwa14_model_urdf();

  REQUIRE(model == model_urdf);
}
