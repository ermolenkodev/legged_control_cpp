#include "legged_control_cpp/urdf.hpp"
#include "queue"

namespace legged_ctrl::urdf_parser {
MultibodyModel parse_urdf(std::string const &filename)
{
  ::urdf::ModelInterfaceSharedPtr const urdfTree = ::urdf::parseURDFFile(filename);
  auto builder = MultibodyModel::create().set_root(urdfTree);

  std::queue<::urdf::LinkConstSharedPtr> link_queue{};

  for (auto const &link : urdfTree->getRoot()->child_links) { link_queue.push(link); }

  while (!link_queue.empty()) {
    ::urdf::LinkConstSharedPtr const link = link_queue.front();
    link_queue.pop();
    for (auto const &child_link : link->child_links) { link_queue.push(child_link); }
    builder.add_link(link);
  }

  return builder;
}
}// namespace legged_ctrl::urdf_parser