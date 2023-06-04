#include <catch2/catch_test_macros.hpp>

#include "legged_control_cpp/spatial.hpp"

TEST_CASE("Spatial rotation", "[rotation]")
{
  using namespace legged_ctrl;

  // how to test that type is not deduced if a non-SO3 type passed?
  STATIC_REQUIRE(std::is_same_v<decltype(spatial_rotation(SO3())), SpatialMatrix>);
}
