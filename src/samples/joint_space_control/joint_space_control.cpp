#include "cmath"
#include "legged_control_cpp/forward_dynamics.hpp"
//#include "legged_control_cpp/jacobian.hpp"
#include "legged_control_cpp/type_aliases.hpp"
#include "legged_control_cpp/urdf.hpp"
#include "optional"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <utility>

using namespace legged_ctrl;

using json = nlohmann::json;

std::optional<json> read_config(spdlog::logger &logger) noexcept
{
  std::ifstream f("src/samples/joint_space_control/config.json");
  try {
    return json::parse(f);
  } catch (json::parse_error const &e) {
    logger.error("Failed to parse config file: %s", e.what());
    return std::nullopt;
  }
}

VectorX get_vector(json const &config, std::string const &key)
{
  std::vector<double> vec = config.at(key);
  return VectorX(Eigen::VectorXd::Map(vec.data(), static_cast<long>(vec.size())));
}

Eigen::Matrix<double, SIX_DIM, SIX_DIM> get_matrix(json const &config, std::string const &key)
{
  std::vector<double> vec = config.at(key);
  return Eigen::Matrix<double, SIX_DIM, SIX_DIM>(
    Eigen::Matrix<double, SIX_DIM, SIX_DIM>::Map(vec.data(), SIX_DIM, SIX_DIM));
}

struct StateWithTimestamp
{
  double time_from_simulation_start_{};
  VectorX state_{};

  StateWithTimestamp(double timestamp, VectorX state) : time_from_simulation_start_(timestamp), state_(std::move(state))
  {}

  friend std::ostream &operator<<(std::ostream &os, StateWithTimestamp const &obj)
  {
    os << obj.time_from_simulation_start_ << ',' << obj.state_.transpose();
    return os;
  }
};

void write_trajectory_to_file(std::vector<StateWithTimestamp> const &trajectory, std::ofstream &file)
{
  for (auto const &sample : trajectory) { file << sample << '\n'; }
}

std::vector<StateWithTimestamp> simulate_joint_space_control(json const &config, spdlog::logger &logger)
{
  VectorX const freq = get_vector(config, "freq");
  VectorX const two_pi_f = 2 * M_PI * freq;
  VectorX const amplitude = get_vector(config, "amp");
  VectorX const two_pi_f_amp = two_pi_f.array() * amplitude.array();
  // TODO fix this
//  VectorX const two_pi_f_squared_amp = two_pi_f.array() * two_pi_f_amp.array();
  VectorX const q0 = get_vector(config, "q0");

  VectorX q = q0;
  VectorX qd = get_vector(config, "qd0");
  VectorX qdd;

  VectorX q_des;
  VectorX qd_des;
//  VectorX qdd_des = VectorX::Zero(q.size());

  MultibodyModel const model = legged_ctrl::urdf_parser::parse_urdf("assets/ur5.urdf");

  VectorX const phi = get_vector(config, "phi");
  double time = 0.0;

  double const simulation_time = config.at("exp_duration");
  double const dt = config.at("dt");
  int const dt_ms = static_cast<int>(dt * 1000);

  VectorX tau;
  auto const kp = get_matrix(config, "kp");
  auto const kd = get_matrix(config, "kd");

  std::vector<StateWithTimestamp> simulated_trajectory;
  simulated_trajectory.reserve(static_cast<size_t>(simulation_time / dt));

  while (time < simulation_time) {
    q_des = q0.array() + amplitude.array() * Eigen::sin(two_pi_f.array() * time + phi.array());
    qd_des = two_pi_f_amp.array() * Eigen::cos(two_pi_f.array() * time + phi.array());
//    qdd_des = two_pi_f_squared_amp.array() * -Eigen::sin(two_pi_f.array() * time + phi.array());

//    auto const J6 = compute_end_effector_frame_jacobian(model, q, LOCAL_WORLD_ALIGNED);
//    auto const J = J6.topRows(3);

    tau = kp * (q_des - q) + kd * (qd_des - qd);

    auto [M, C] = crba(model, SystemConfiguration{ q, qd }, ExternalForces{ VectorX::Zero(q.size()) });
    auto M_inv = M.inverse();

    qdd = M_inv * (tau - C);
    qd = qd + qdd * dt;
    q = q + dt * qd + 0.5 * dt * dt * qdd; // NOLINT

    simulated_trajectory.emplace_back(time, q);
    time += dt;
    std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  }

  logger.info("Done");

  return simulated_trajectory;
}

int main()
{
  auto logger = *spdlog::stdout_color_mt("lcc_joint_space_control");
  logger.set_level(spdlog::level::debug);

  auto optional_config = read_config(logger);

  if (!optional_config) {
    logger.error("Failed to read config file");
    return 1;
  }

  std::vector<StateWithTimestamp> simulated_trajectory;
  try {
    simulated_trajectory = simulate_joint_space_control(*optional_config, logger);
  } catch (...) {
    logger.error("Failed to simulate joint space control");
    return 1;
  }

  std::ofstream outFile("joint_space_control.log");
  write_trajectory_to_file(simulated_trajectory, outFile);
  outFile.close();

  return 0;
}
