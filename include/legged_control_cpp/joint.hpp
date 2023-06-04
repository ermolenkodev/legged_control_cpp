#ifndef JOINT_HPP
#define JOINT_HPP

#include "crtp.hpp"
#include "type_aliases.hpp"
#include "spatial.hpp"
#include "variant"

namespace legged_ctrl {
enum class JointAxis { X, Y, Z, UNALIGNED };

template<typename Derived> class JointMetadata : public Crtp<Derived>
{
public:
  SpatialMatrix joint_transform(double theta) { return this->derived()->joint_transform_impl(theta); }

  SpatialVector screw_axis() { return this->derived()->screw_axis_impl(); }
};

class RevoluteJoint : public JointMetadata<RevoluteJoint>
{
public:
  explicit RevoluteJoint(JointAxis axis) : axis_(axis) {}

  SpatialMatrix joint_transform_impl(double theta)
  {
    switch (axis_) {
    case JointAxis::X:
      return rotx(theta);
    case JointAxis::Y:
      return roty(theta);
    case JointAxis::Z:
      return rotz(theta);
    case JointAxis::UNALIGNED:
      throw std::runtime_error("Not implemented yet");
    }
  }

  SpatialVector screw_axis_impl()
  {
    SpatialVector result;
    result.setZero();
    result(static_cast<int>(axis_)) = 1;
    return result;
  }

private:
  JointAxis axis_;
};

using JointModelVariant = std::variant<RevoluteJoint>;
}// namespace legged_ctrl

#endif
