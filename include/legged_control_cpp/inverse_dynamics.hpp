#ifndef LCC_INVERSE_DYNAMICS_HPP
#define LCC_INVERSE_DYNAMICS_HPP

#include <utility>

#include "external_forces.hpp"
#include "model.hpp"
#include "optional"
#include "tuple"
#include "type_aliases.hpp"
#include "unordered_map"
#include "vector"

namespace legged_ctrl {

struct RNEAExtendedOutput
{
  IndexedSpatialVectors V_;
  IndexedSpatialVectors A_;
  IndexedSpatialVectors F_;
  std::vector<SpatialMatrix> Xup_;
  std::vector<SpatialVector> S_;
  VectorX manipulator_eq_lhs_;

  RNEAExtendedOutput(IndexedSpatialVectors V,
    IndexedSpatialVectors A,
    IndexedSpatialVectors F,
    std::vector<SpatialMatrix> Xup,
    std::vector<SpatialVector> S,
    VectorX manipulator_eq_lhs)
    : V_(std::move(V)), A_(std::move(A)), F_(std::move(F)), Xup_(std::move(Xup)), S_(std::move(S)),
      manipulator_eq_lhs_(std::move(manipulator_eq_lhs))
  {}

  VectorX tau() const;

  VectorX nle() const;

  VectorX g() const;
};

RNEAExtendedOutput rnea_with_extended_output(MultibodyModel const &model,
  SystemConfiguration const &systemConfiguration,
  ExternalForces const &externalForces = ExternalForces::none(),
  Vector3 const &gravity = Vector3{ 0, 0, G });

VectorX rnea(MultibodyModel const &model,
  SystemConfiguration const &systemConfiguration,
  ExternalForces const &externalForces = ExternalForces::none(),
  Vector3 const &gravity = Vector3{ 0, 0, G });

namespace details {
  RNEAExtendedOutput rnea_with_extended_output(MultibodyModel const &model,
    VectorX const &q,
    VectorX const &qd,
    VectorX const &qdd,
    bool skipCoriolisEffects,
    ExternalForces const &externalForces,
    Vector3 const &gravity = Vector3{ 0, 0, G });
}

}// namespace legged_ctrl

#endif
