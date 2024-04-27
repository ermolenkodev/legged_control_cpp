#ifndef LCC_MULTIBODY_MODEL_BUILDER_HPP
#define LCC_MULTIBODY_MODEL_BUILDER_HPP

#include "logging.hpp"
#include "map"
#include "memory"
#include "model.hpp"
#include "spdlog/logger.h"
#include "spdlog/spdlog.h"
#include "type_aliases.hpp"

namespace legged_ctrl {

class MultibodyModelBuilderBase
{
public:
  virtual MultibodyModelBuilderBase &set_logger(spdlog::logger logger);
  [[nodiscard]] MultibodyModel build() const { return std::move(*model_); }
  virtual ~MultibodyModelBuilderBase() = default;

protected:
  MultibodyModelBuilderBase(ModelPtr model, LinkNameToIndexMapPtr link_name_to_idx);
  MultibodyModelBuilderBase(ModelPtr model, LinkNameToIndexMapPtr link_name_to_idx, LoggerPtr logger);
  MultibodyModelBuilderBase(MultibodyModelBuilderBase &&) = default;
  MultibodyModelBuilderBase &operator=(MultibodyModelBuilderBase &&) = default;
  MultibodyModelBuilderBase(MultibodyModelBuilderBase const &) = default;
  MultibodyModelBuilderBase &operator=(MultibodyModelBuilderBase const &) = default;

  ModelPtr model_{};
  LinkNameToIndexMapPtr link_name_to_idx_{};
  LoggerPtr logger_{ null_logger() };
};

class MultibodyModelBuilderWithoutRoot : public MultibodyModelBuilderBase
{
public:
  MultibodyModelBuilderWithoutRoot();
  MultibodyModelBuilder set_root(::urdf::ModelInterfaceSharedPtr const &urdf_tree);
  MultibodyModelBuilderWithoutRoot &set_logger(spdlog::logger logger) override;
};

class MultibodyModelBuilder : public MultibodyModelBuilderBase
{
public:
  MultibodyModelBuilder &add_link(::urdf::LinkConstSharedPtr const &link);
  MultibodyModelBuilder &set_logger(spdlog::logger logger) override;

private:
  MultibodyModelBuilder(ModelPtr model, LinkNameToIndexMapPtr link_name_to_idx, LoggerPtr logger);
  friend class MultibodyModelBuilderWithoutRoot;
};

namespace builder::details {
  using AxisVec = std::pair<std::array<double, 3>, JointAxis>;

  JointAxis determine_joint_axis(std::array<double, 3> const &vec);
}// namespace builder::details


}// namespace legged_ctrl

#endif
