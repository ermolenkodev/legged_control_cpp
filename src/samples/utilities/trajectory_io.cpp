#include "trajectory_io.hpp"

std::ostream &operator<<(std::ostream &os, const StateWithTimestamp &obj)
{
  os << obj.time_from_simulation_start_ << ',' << obj.state_.transpose();
  return os;
}

void write_trajectory_to_file(std::vector<StateWithTimestamp> const &trajectory, std::ofstream &file)
{
  for (auto const &sample : trajectory) { file << sample << '\n'; }
}
