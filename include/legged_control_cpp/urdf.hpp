#ifndef LEGGED_CTRL_URDF_HPP
#define LEGGED_CTRL_URDF_HPP

#include "model.hpp"
#include "builder.hpp"
#include "urdf_parser/urdf_parser.h"
#include "urdf_model/model.h"

namespace legged_ctrl::urdf_parser {
  MultibodyModel parse_urdf(std::string const &filename);
}// namespace legged_ctrl

#endif