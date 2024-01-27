#include "legged_control_cpp/spatial.hpp"

namespace legged_ctrl {

// clang-format off
SO3 Rx(double theta) {
  double const c = std::cos(theta);
  double const s = std::sin(theta);

  SO3 R;
  R << 1, 0, 0,
       0, c, s,
       0, -s, c;
  return R;
}

SO3 Ry(double theta) {
  double const c = std::cos(theta);
  double const s = std::sin(theta);

  SO3 R;
  R << c, 0, -s,
       0, 1, 0,
       s, 0, c;
  return R;
}

SO3 Rz(double theta) {
  double const c = std::cos(theta);
  double const s = std::sin(theta);

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

SpatialMatrix translation_part(SpatialMatrix const &X)
{
  SpatialMatrix X_translation = X;
  SO3 const &R = X.template topLeftCorner<3, 3>();
  X_translation.bottomLeftCorner<3, 3>() =  X.template bottomLeftCorner<3, 3>() * R.transpose();
  X_translation.topLeftCorner<3, 3>().setIdentity();
  X_translation.bottomRightCorner<3, 3>().setIdentity();

  return X_translation;
}

SE3 translation_part(SE3 const &T)
{
  SE3 T_translation = T;
  T_translation.template topLeftCorner<3, 3>().setIdentity();

  return T_translation;
}

DiagonalMatrix diag(JointSpaceMatrix const &M)
{
  DiagonalMatrix D{ M.rows() };
  D.diagonal() = M.diagonal();

  return D;
}

}// namespace legged_ctrl
