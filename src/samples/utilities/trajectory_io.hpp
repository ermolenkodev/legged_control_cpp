#include "legged_control_cpp/type_aliases.hpp"
#include <fstream>
#include <iostream>

using namespace legged_ctrl;

struct StateWithTimestamp
{
  double time_from_simulation_start_{};
  VectorX state_{};

  StateWithTimestamp(double timestamp, VectorX state) : time_from_simulation_start_(timestamp), state_(std::move(state))
  {}

  friend std::ostream &operator<<(std::ostream &os, StateWithTimestamp const &obj);
};

void write_trajectory_to_file(std::vector<StateWithTimestamp> const &trajectory, std::ofstream &file);
