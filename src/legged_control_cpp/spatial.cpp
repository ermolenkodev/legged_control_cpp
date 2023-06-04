#include "legged_control_cpp/spatial.hpp"

namespace legged_ctrl {
// clang-format off
SO3 Rx(double theta) {
  double c = std::cos(theta);
  double s = std::sin(theta);

  SO3 R;
  R << 1, 0, 0,
       0, c, s,
       0, -s, c;
  return R;
}

SO3 Ry(double theta) {
  double c = std::cos(theta);
  double s = std::sin(theta);

  SO3 R;
  R << c, 0, -s,
       0, 1, 0,
       s, 0, c;
  return R;
}

SO3 Rz(double theta) {
  double c = std::cos(theta);
  double s = std::sin(theta);

  SO3 R;
  R << c, s, 0,
      -s, c, 0,
       0, 0, 1;
  return R;
}
// clang-format on

SpatialMatrix rotx(double theta) { return spatial_rotation(Rx(theta)); }

SpatialMatrix roty(double theta) { return spatial_rotation(Ry(theta)); }

SpatialMatrix rotz(double theta) { return spatial_rotation(Rz(theta)); }
}// namespace legged_ctrl
