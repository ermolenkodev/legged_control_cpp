#ifndef MULTIBODY_MODEL_BUILDER_HPP
#define MULTIBODY_MODEL_BUILDER_HPP

#include "map"
#include "model.hpp"
#include "memory"

namespace legged_ctrl {

using LinkNameToIndexMap = std::map<std::string, int>;
using ModelPtr = std::shared_ptr<MultibodyModel>;
using LinkNameToIndexMapPtr = std::shared_ptr<LinkNameToIndexMap>;

class MultibodyModelBuilderBase
{
public:
  // Is it good idea to move here?
  operator MultibodyModel() const { return std::move(*model); }

protected:
  MultibodyModelBuilderBase(ModelPtr model_, LinkNameToIndexMapPtr link_name_to_idx_);
  // Is it possible to implement it without heap allocations?
  //  MultibodyModel &model;
  //  LinkNameToIndexMap &link_name_to_idx;
  ModelPtr model;
  LinkNameToIndexMapPtr link_name_to_idx;
};

class MultibodyModelBuilderWithoutRoot : public MultibodyModelBuilderBase
{
public:
  MultibodyModelBuilderWithoutRoot();
  MultibodyModelBuilder set_root(::urdf::ModelInterfaceSharedPtr const &urdf_tree);
//private:
  //  MultibodyModel result_model{};
  //  LinkNameToIndexMap name_to_idx{};
};

class MultibodyModelBuilder : public MultibodyModelBuilderBase
{
public:
  MultibodyModelBuilder(ModelPtr  model_, LinkNameToIndexMapPtr link_name_to_idx_);
  MultibodyModelBuilder &add_link(::urdf::LinkConstSharedPtr const &link);
};
}// namespace legged_ctrl

#endif