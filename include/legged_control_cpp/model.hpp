#ifndef MULTIBODY_MODEL_HPP
#define MULTIBODY_MODEL_HPP

#include "joint.hpp"
#include "memory"
#include "optional"
#include "spdlog/logger.h"
#include "type_aliases.hpp"
#include "urdf_model/model.h"
#include "urdf_parser/urdf_parser.h"
#include "vector"

namespace legged_ctrl {

class MultibodyModelBuilder;
class MultibodyModelBuilderWithoutRoot;
namespace mjxml {
  class MjxmlModelBuilderWithoutRoot;
}

}// namespace legged_ctrl

namespace legged_ctrl {

struct MultibodyModel
{
public:
  std::string name{};
  int n_bodies{ 0 };
  std::vector<JointModelVariant> joints{};
  std::vector<int> parent{};
  std::vector<SpatialMatrix> X_tree{};
  std::vector<SpatialMatrix> I{};
  std::optional<SE3> nTee{ std::nullopt };

  static MultibodyModelBuilderWithoutRoot create_from_urdf();
  static mjxml::MjxmlModelBuilderWithoutRoot create_from_mjxml();
  friend class MultibodyModelBuilderWithoutRoot;
  friend class MultibodyModelBuilder;

  friend std::ostream &operator<<(std::ostream &os, MultibodyModel const &model);

  [[nodiscard]] std::optional<SE3> const &get_ee_transform() const { return nTee; }

  [[nodiscard]] int num_bodies() const { return n_bodies; }

  [[nodiscard]] std::vector<int> get_parent_idxs() const { return parent; }

  bool operator==(MultibodyModel const &other) const;
};

const double G = -9.81;

struct SystemConfiguration
{
  VectorX q{};
  VectorX qd{};
  VectorX qdd{};

  explicit SystemConfiguration(VectorX q_) : q(std::move(q_)), qd(VectorX::Zero(q.size())), qdd(VectorX::Zero(q.size()))
  {}

  SystemConfiguration(VectorX q_, VectorX qd_) : q(std::move(q_)), qd(std::move(qd_)), qdd(VectorX::Zero(q.size())) {}

  SystemConfiguration(VectorX q_, VectorX qd_, VectorX qdd_)
    : q(std::move(q_)), qd(std::move(qd_)), qdd(std::move(qdd_))
  {}
};

struct ExternalForces
{
  std::optional<IndexedSpatialVectors> f_ext{ std::nullopt };
  std::optional<SpatialVector> f_tip{ std::nullopt };

  ExternalForces() = default;

  // should this constructor be explicit?
  explicit ExternalForces(IndexedSpatialVectors f_ext_) : f_ext(std::move(f_ext_)) {}

  explicit ExternalForces(SpatialVector f_tip_) : f_tip(std::move(f_tip_)) {}

  ExternalForces(IndexedSpatialVectors f_ext_, SpatialVector f_tip_)
    : f_ext(std::move(f_ext_)), f_tip(std::move(f_tip_))
  {}

  static ExternalForces none() { return {}; }
};

}// namespace legged_ctrl

#endif
