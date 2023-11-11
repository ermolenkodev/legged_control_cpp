#include "legged_control_cpp/urdf.hpp"
#include "queue"
#include "spdlog/sinks/stdout_color_sinks.h"

namespace legged_ctrl::urdf_parser {

MultibodyModel parse_urdf(std::string const &filename, std::optional<spdlog::logger> const &logger)
{
  ::urdf::ModelInterfaceSharedPtr const urdfTree = ::urdf::parseURDFFile(filename);
  auto builder = MultibodyModel::create().set_root(urdfTree);

  if (logger) { builder.set_logger(*logger); }

  std::queue<::urdf::LinkConstSharedPtr> link_queue{};

  for (auto const &link : urdfTree->getRoot()->child_links) { link_queue.push(link); }

  while (!link_queue.empty()) {
    ::urdf::LinkConstSharedPtr const link = link_queue.front();
    link_queue.pop();
    for (auto const &child_link : link->child_links) { link_queue.push(child_link); }
    builder.add_link(link);
  }

  return builder.build();
}

}// namespace legged_ctrl::urdf_parser
