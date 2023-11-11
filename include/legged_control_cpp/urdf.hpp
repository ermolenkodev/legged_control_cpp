#ifndef LEGGED_CTRL_URDF_HPP
#define LEGGED_CTRL_URDF_HPP

#include "builder.hpp"
#include "model.hpp"
#include "urdf_model/model.h"
#include "urdf_parser/urdf_parser.h"

namespace legged_ctrl::urdf_parser {

MultibodyModel parse_urdf(std::string const &filename, std::optional<spdlog::logger> const &logger = std::nullopt);

}// namespace legged_ctrl::urdf_parser

#endif