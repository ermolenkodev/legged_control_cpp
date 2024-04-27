#ifndef LCC_MUJOCO_API_HPP
#define LCC_MUJOCO_API_HPP

#include "legged_control_cpp/type_aliases.hpp"
#include "simulate.h"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mujoco/mujoco.h>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include "array_safety.h"
#include "glfw_adapter.h"
#include "legged_control_cpp/forward_dynamics.hpp"
#include "legged_control_cpp/forward_kinematics.hpp"
#include "legged_control_cpp/jacobian.hpp"
#include "legged_control_cpp/mjxml/mjxml.hpp"
#include "legged_control_cpp/model.hpp"
#include "legged_control_cpp/logging.hpp"
#include "simulate.h"
#include <mujoco/mujoco.h>
#include <spdlog/sinks/stdout_color_sinks.h>


namespace task_space_control {

using namespace legged_ctrl;
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

struct LowLevelMujocoObjects
{
  std::unique_ptr<mjvCamera> cam{};
  std::unique_ptr<mjvOption> opt{};
  std::unique_ptr<mjvPerturb> pert{};
  std::unique_ptr<mj::Simulate> sim{};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m{nullptr, mj_deleteModel};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> d{nullptr, mj_deleteData};
  std::vector<double> ctrlnoise{};
};

class Mujoco
{
public:
  using Clock = std::chrono::steady_clock;
  using Seconds = std::chrono::duration<double>;
  using TimePoint = std::chrono::steady_clock::time_point;

  Mujoco();

  bool load_model(std::string const &model_path, LoggerPtr const &logger = null_logger());
  std::recursive_mutex& mutex() const;

  class TimeApi
  {
  public:
    explicit TimeApi(LowLevelMujocoObjects &mujoco_objects) : mujoco_objects(mujoco_objects) {}
    double get_sim_time() const;
    double get_slowdown_factor() const;
    bool is_simulation_speed_changed() const;
    void mark_simulation_speed_as_unchanged();
    void mark_simulation_speed_as_changed();
    bool is_cpu_and_sim_time_out_of_sync(TimePoint const& cpu_iteration_start, double sim_iteration_start) const;
    void synchronize_time(TimePoint const& cpu_iteration_start, double sim_iteration_start);
    bool is_sim_behind_cpu() const;
    bool is_sim_within_refresh_time(TimePoint const &cpu_iteration_start, int refresh_rate) const;
    void measure_slowdown_factor(TimePoint const& cpu_iteration_start, double sim_iteration_start);
  private:
    const double sim_refresh_fraction{0.7};
    const double sync_misalign_threshold{0.1};
    TimePoint cpu_sync_time{};
    double sim_sync_time{0};
    LowLevelMujocoObjects &mujoco_objects;
  };

  class State
  {
  public:
    explicit State(LowLevelMujocoObjects &mujoco_objects) : mujoco_objects(mujoco_objects) {}
    VectorX get_joint_positions() const;
    VectorX get_joint_velocities() const;
    Vector3 get_mocap_position(int mocap_id) const;
    SO3 get_mocap_orientation(int mocap_id) const;
  private:
    LowLevelMujocoObjects &mujoco_objects;
  };

  class Gui
  {
  public:
    explicit Gui(LowLevelMujocoObjects &mujoco_objects) : mujoco_objects(mujoco_objects) {}
    bool is_simulation_paused() const;
    bool is_exit_requested() const;
    void enter_render_loop();
    int refresh_rate() const;
    void save_current_state_to_history();
    bool is_ctrl_noise_enabled() const;
    void pause_simulation();
  private:
    LowLevelMujocoObjects &mujoco_objects;
  };

  class Simulator
  {
  public:
    explicit Simulator(LowLevelMujocoObjects &mujoco_objects) : mujoco_objects(mujoco_objects) {}
    void step_simulation();
    void forward();
    void apply_control_torques(const VectorX &tau, const std::vector<int> &actuator_ids);
    void set_ctrl_noise(std::vector<double> noise);
    void init_ctrl_noise_with_stddev();
  private:
    LowLevelMujocoObjects &mujoco_objects;
  };

  class Scene
  {
  public:
    explicit Scene(LowLevelMujocoObjects &mujoco_objects) : mujoco_objects(mujoco_objects) {}
    int get_mocap_id(std::string const &name) const;
    [[nodiscard]] int get_site_id(std::string const &name) const;
    void set_state_from_keyframe(std::string const &keyframe_name);

    int get_actuator_id(std::string const &name) const;
    std::vector<int> get_actuator_ids(std::vector<std::string> const &joint_names) const;
  private:
    LowLevelMujocoObjects &mujoco_objects;
  };

  Scene scene{mujoco_objects};
  State state{mujoco_objects};
  Gui gui{mujoco_objects};
  Simulator simulator{mujoco_objects};
  TimeApi time_api{mujoco_objects};
private:
  LowLevelMujocoObjects mujoco_objects{};
};

}// namespace task_space_control

#endif// LCC_MUJOCO_API_HPP
