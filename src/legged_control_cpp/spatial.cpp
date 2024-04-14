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
  X_translation.bottomLeftCorner<3, 3>() = X.template bottomLeftCorner<3, 3>() * R.transpose();
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

Vector3 vector_from_SO3(SkewSymmetric const &S)
{
  Vector3 v;
  v << S(2, 1), S(0, 2), S(1, 0);

  return v;
}

namespace {
  bool near_zero(double value, double tolerance = 1e-6) { return std::abs(value) < tolerance; }// NOLINT
}// namespace

std::pair<double, Vector3> angle_axis_from_SO3(SO3 const &R)
{
  double const y =
    std::sqrt(std::pow(R(2, 1) - R(1, 2), 2) + std::pow(R(0, 2) - R(2, 0), 2) + std::pow(R(1, 0) - R(0, 1), 2));
  double const x = R(0, 0) + R(1, 1) + R(2, 2) - 1;
  double delta_theta = std::atan2(y, x);

  Vector3 r_hat;
  if (near_zero(delta_theta)) {
    delta_theta = 0;
    r_hat << 0, 0, 1;
  } else if (near_zero(R.trace() + 1)) {// approx -1
    delta_theta = M_PI;
    double scale;// NOLINT
    if (!near_zero(1 + R(2, 2))) {
      scale = 1.0 / std::sqrt(2 * (1 + R(2, 2)));
      r_hat = scale * Vector3(R(0, 2), R(1, 2), 1 + R(2, 2));
    } else if (!near_zero(1 + R(1, 1))) {
      scale = 1.0 / std::sqrt(2 * (1 + R(1, 1)));
      r_hat = scale * Vector3(R(0, 1), 1 + R(1, 1), R(2, 1));
    } else {
      scale = 1.0 / std::sqrt(2 * (1 + R(0, 0)));
      r_hat = scale * Vector3(1 + R(0, 0), R(1, 0), R(2, 0));
    }
  } else {
    r_hat << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
    r_hat /= (2 * std::sin(delta_theta));
  }
  r_hat.normalize();

  return { delta_theta, r_hat };
}

}// namespace legged_ctrl
