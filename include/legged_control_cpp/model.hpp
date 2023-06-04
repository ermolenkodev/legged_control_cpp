#ifndef MULTIBODY_MODEL_HPP
#define MULTIBODY_MODEL_HPP

#include "memory"
#include "urdf_model/model.h"
#include "urdf_parser/urdf_parser.h"
#include "vector"
#include "type_aliases.hpp"
#include "joint.hpp"

namespace legged_ctrl {
  class MultibodyModelBuilder;
  class MultibodyModelBuilderWithoutRoot;
}

namespace legged_ctrl {
  class MultibodyModel {
  public:
    static MultibodyModelBuilderWithoutRoot create();
    friend class MultibodyModelBuilderWithoutRoot;
    friend class MultibodyModelBuilder;

    friend std::ostream& operator<<(std::ostream& os, MultibodyModel const& model);
  private:
    std::string name{};
    int n_bodies{0};
    std::vector<JointModelVariant> joints{};
    std::vector<int> parent{};
    std::vector<SpatialMatrix> X_tree{};
    std::vector<SpatialMatrix> I{};
  };

}// namespace legged_ctrl

#endif
