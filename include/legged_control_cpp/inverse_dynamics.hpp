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
  VectorX tau_;

  RNEAExtendedOutput(IndexedSpatialVectors V,
    IndexedSpatialVectors A,
    IndexedSpatialVectors F,
    std::vector<SpatialMatrix> Xup,
    std::vector<SpatialVector> S,
    VectorX tau)
    : V_(std::move(V)), A_(std::move(A)), F_(std::move(F)), Xup_(std::move(Xup)), S_(std::move(S)), tau_(std::move(tau))
  {}
};

RNEAExtendedOutput rnea_with_extended_output(PhysicsDescription const &physicsDescription,
  SystemConfiguration const &systemConfiguration,
  ExternalForces const &externalForces);

VectorX rnea(PhysicsDescription const &physicsDescription,
  SystemConfiguration const &systemConfiguration,
  ExternalForces const &externalForces);
}// namespace legged_ctrl

#endif
