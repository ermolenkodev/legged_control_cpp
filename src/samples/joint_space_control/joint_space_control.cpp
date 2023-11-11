#include "cmath"
#include "legged_control_cpp/type_aliases.hpp"
#include "legged_control_cpp/urdf.hpp"
#include "legged_control_cpp/jacobian.hpp"
#include "optional"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using namespace legged_ctrl;

using json = nlohmann::json;

std::optional<json> read_config() noexcept
{
  std::ifstream f("src/samples/joint_space_control/config.json");
  try {
    return json::parse(f);
  } catch (json::parse_error const &e) {
    std::cerr << "Failed to parse config file:\n" << e.what() << '\n';
    return std::nullopt;
  }
}

VectorX get_vector(json const &config, std::string const &key)
{
  std::vector<double> vec = config.at(key);
  return VectorX(Eigen::VectorXd::Map(vec.data(), static_cast<long>(vec.size())));
}

void simulate_joint_space_control(json const &config)
{
  VectorX const freq = get_vector(config, "freq");
  VectorX const two_pi_f = 2 * M_PI * freq;
  VectorX const amplitude = get_vector(config, "amp");
  VectorX const two_pi_f_amp = two_pi_f.array() * amplitude.array();
  // TODO fix this
  VectorX const two_pi_f_squared_amp = two_pi_f.array() * two_pi_f_amp.array();
  VectorX q0 = get_vector(config, "q0");
  VectorX const q = q0;
  VectorX const qd = get_vector(config, "qd0");
  VectorX const qdd = get_vector(config, "qdd0");

  VectorX q_des = q;
  VectorX qd_des = VectorX::Zero(q.size());
  VectorX qdd_des = VectorX::Zero(q.size());

  MultibodyModel const model = legged_ctrl::urdf_parser::parse_urdf("assets/ur5.urdf");

  auto start_time = std::chrono::high_resolution_clock::now();
  VectorX const phi = get_vector(config, "phi");
  double time = 0.0;

  double const simulation_time = config.at("exp_duration");
  int const dt = config.at("dt");

  while (time < simulation_time) {
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> const elapsed_time = end_time - start_time;
    time = elapsed_time.count();

    q_des = q0.array() + amplitude.array() * Eigen::sin(two_pi_f.array() * time + phi.array());
    qd_des = two_pi_f_amp.array() * Eigen::cos(two_pi_f.array() * time + phi.array());
    qdd_des = two_pi_f_squared_amp.array() * -Eigen::sin(two_pi_f.array() * time + phi.array());

    std::this_thread::sleep_for(std::chrono::milliseconds(dt));

    std::cout << "q_des: " << q_des.transpose() << '\n';
    std::cout << "qd_des: " << qd_des.transpose() << '\n';
    std::cout << "qdd_des: " << qdd_des.transpose() << "\n\n";

    auto J = compute_end_effector_frame_jacobian(model, q, LOCAL_WORLD_ALIGNED);

    std::cout << "J:\n" << J << '\n';
  }
}

int main()
{
  auto optional_config = read_config();

  if (!optional_config) {
    std::cerr << "Failed to read config file" << '\n';
    return 1;
  }

  try {
    simulate_joint_space_control(*optional_config);
  } catch (...) {
    std::cerr << "Failed to simulate joint space control" << '\n';
    return 1;
  }

  return 0;
}
