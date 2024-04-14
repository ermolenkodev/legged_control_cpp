#ifndef LCC_SPATIAL_HPP
#define LCC_SPATIAL_HPP

#include "type_aliases.hpp"

namespace legged_ctrl {

SO3 Rx(double theta);

SO3 Ry(double theta);

SO3 Rz(double theta);

template<typename SO3_> using IsSO3 = std::enable_if_t<std::is_same_v<SO3, std::decay_t<SO3_>>, SO3_>;

template<typename SO3_, typename = IsSO3<SO3_>> SpatialMatrix spatial_rotation(SO3_ &&R)
{
  SpatialMatrix X;
  X.setZero();
  // looks like in Eigen, assignment operation for matrix expressions does not make use of move semantics,
  // so there is no advantage to using std::forward in this situation. However, it should not hurt either.
  X.template topLeftCorner<3, 3>() = R;
  X.template bottomRightCorner<3, 3>() = std::forward<SO3_>(R);

  return X;
}

SpatialMatrix rotx(double theta);

SpatialMatrix roty(double theta);

SpatialMatrix rotz(double theta);

template<typename Vector3_>
using IsVector3 = std::enable_if_t<std::is_same_v<Vector3, std::decay_t<Vector3_>>, Vector3_>;

template<typename Vector3_, typename = IsVector3<Vector3_>> SkewSymmetric so3(Vector3_ &&v)
{
  SkewSymmetric S;
  // clang-format off
  S << 0, -v(2), v(1),
       v(2), 0, -v(0),
      -v(1), v(0), 0; // cppcheck-suppress constStatement
  // clang-format on

  return S;
}

template<typename SE3_> using IsSE3 = std::enable_if_t<std::is_same_v<SE3, std::decay_t<SE3_>>, SE3_>;

template<typename SE3_, typename = IsSE3<SE3_>> SpatialMatrix Ad(SE3_ &&T)
{
  SpatialMatrix X;
  X.setZero();
  SO3 R = T.template topLeftCorner<3, 3>();
  X.template topLeftCorner<3, 3>() = R;
  X.template bottomRightCorner<3, 3>() = R;
  Vector3 p = T.template topRightCorner<3, 1>();
  X.template bottomLeftCorner<3, 3>() = so3(p) * R;

  return X;
}

// How to deal with function of multiple arguments and SFINAE? It looks too verbose to me.
// why type deduction did not work here?
template<typename RotationalInertia_>
using IsRotationalInertia =
  std::enable_if_t<std::is_same_v<RotationalInertia, std::decay_t<RotationalInertia_>>, RotationalInertia_>;

template<typename RotationalInertia_, typename Vector3_>
SpatialMatrix I_from_rotinertia_about_com(RotationalInertia_ &&rotI_C_C, Vector3_ &&p_C_i, double mass)
{
  static_assert(std::is_same_v<RotationalInertia, std::decay_t<RotationalInertia_>>,
    "rotational inertia must be of type RotationalInertia");
  static_assert(std::is_same_v<Vector3, std::decay_t<Vector3_>>, "vector must be of type Vector3");

  SpatialMatrix I;
  I.setZero();
  SkewSymmetric const com_so3 = so3(p_C_i);
  SkewSymmetric const com_so3_T = com_so3.transpose();
  I.template topLeftCorner<3, 3>() = rotI_C_C + mass * com_so3 * com_so3_T;
  I.template topRightCorner<3, 3>() = mass * com_so3;
  I.template bottomLeftCorner<3, 3>() = mass * com_so3_T;
  I.template bottomRightCorner<3, 3>() = mass * Matrix3::Identity();

  return I;
}

template<typename Vector3_, typename = IsVector3<Vector3_>> SpatialVector spatial_gravity(Vector3_ &&gravity)
{
  SpatialVector g;
  g.setZero();
  g.template bottomRows<3>() = std::forward<Vector3_>(gravity);
  return g;
}

template<typename SpatialVector_>
using IsSpatialVector = std::enable_if_t<std::is_same_v<SpatialVector, std::decay_t<SpatialVector_>>, SpatialVector_>;

template<typename SpatialVector_, typename = IsSpatialVector<SpatialVector_>> SpatialMatrix Vx(SpatialVector_ &&V)
{
  SpatialMatrix X;
  X.setZero();
  Vector3 const omega = V.template topRows<3>();
  Vector3 const v = V.template bottomRows<3>();
  SkewSymmetric const omega_so3 = so3(omega);// NOLINT
  X.template topLeftCorner<3, 3>() = omega_so3;
  X.template bottomLeftCorner<3, 3>() = so3(v);
  X.template bottomRightCorner<3, 3>() = omega_so3;

  return X;
}


template<typename SpatialVector_, typename = IsSpatialVector<SpatialVector_>> SpatialMatrix Vx_star(SpatialVector_ &&V)
{
  return -Vx(std::forward<SpatialVector_>(V)).transpose();
}

template<typename SE3_, typename = IsSE3<SE3_>> SE3 Tinv(SE3_ &&T)
{
  SE3 T_inv;
  T_inv.setZero();
  auto const Rt = T.template topLeftCorner<3, 3>().transpose();
  auto const p = T.template topRightCorner<3, 1>();
  T_inv.template topLeftCorner<3, 3>() = Rt;
  T_inv.template topRightCorner<3, 1>() = -Rt * p;
  T_inv(3, 3) = 1.0;

  return T_inv;
}

SpatialMatrix translation_part(SpatialMatrix const &X);

SE3 translation_part(SE3 const &T);

DiagonalMatrix diag(JointSpaceMatrix const &M);

Vector3 vector_from_SO3(SkewSymmetric const &S);

std::pair<double, Vector3> angle_axis_from_SO3(SO3 const &R);

}// namespace legged_ctrl

#endif
