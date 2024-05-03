#include "mujoco_api.hpp"

namespace task_space_control {
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic, readability-make-member-function-const)

std::recursive_mutex &Mujoco::mutex() const { return mujoco_objects.sim->mtx; }

Mujoco::Mujoco()
{
  mujoco_objects.cam = std::make_unique<mjvCamera>();
  mjv_defaultCamera(mujoco_objects.cam.get());
  mujoco_objects.opt = std::make_unique<mjvOption>();
  mjv_defaultOption(mujoco_objects.opt.get());
  mujoco_objects.pert = std::make_unique<mjvPerturb>();
  mjv_defaultPerturb(mujoco_objects.pert.get());
  mujoco_objects.sim = std::make_unique<mj::Simulate>(std::make_unique<mj::GlfwAdapter>(),
    mujoco_objects.cam.get(),
    mujoco_objects.opt.get(),
    mujoco_objects.pert.get(),
    /* is_passive = */ false);
}

namespace {
  constexpr auto MJB_FILE_EXTENSION = ".mjb";

  std::string get_file_extension(std::string const &filename)
  {
    auto dot_position = filename.rfind('.');

    if (dot_position != std::string::npos) { return filename.substr(dot_position); }

    return {};
  }

  bool is_empty(char const *msg) { return msg == nullptr || *msg == '\0'; }

  void remove_newline_at_end(char *str)
  {
    if (!is_empty(str)) {
      std::size_t const length = std::strlen(str);
      if (str[length - 1] == '\n') { str[length - 1] = '\0'; }
    }
  }

  bool is_mjb_file(std::string const &file) { return get_file_extension(file) == MJB_FILE_EXTENSION; }

  std::pair<mjModel *, std::string> load_model_from_file(std::string const &file)
  {
    if (file.empty()) { return { nullptr, "Provided filename is empty" }; }

    const int max_error_length = 1024;
    std::unique_ptr<char[]> const load_error{ new char[max_error_length] };// NOLINT

    mjModel *model = is_mjb_file(file) ? mj_loadModel(file.c_str(), nullptr)
                                       : mj_loadXML(file.c_str(), nullptr, load_error.get(), max_error_length);

    // remove trailing newline character from error message
    char *error_msg = load_error.get();
    remove_newline_at_end(error_msg);

    if (model == nullptr and is_mjb_file(file)) { return { nullptr, "Could not load .mjb file" }; }
    if (model == nullptr) { return { nullptr, error_msg }; }
    if (!is_empty(error_msg)) { return { model, error_msg }; }

    return { model, "" };
  }
}// namespace

bool Mujoco::load_model(const std::string &model_path, LoggerPtr const &logger)
{
  mujoco_objects.sim->LoadMessage(model_path.c_str());
  auto [model, error_msg] = load_model_from_file(model_path);

  if (model == nullptr) {
    logger->error("Failed to load model: {}", error_msg);
    return false;
  }

  if (!error_msg.empty()) {
    logger->warn("Model compiled, but simulation warning (paused):\n  {}", error_msg);
    // forward() below will print the warning message
    gui.pause_simulation();
  }
  mujoco_objects.m = { model, mj_deleteModel };

  {
    const std::unique_lock<std::recursive_mutex> lock(mujoco_objects.sim->mtx);
    mujoco_objects.d = { mj_makeData(mujoco_objects.m.get()), mj_deleteData };
  }

  if (!mujoco_objects.d) {
    mujoco_objects.sim->LoadMessageClear();
    logger->error("Failed to create mjData object");

    return false;
  }

  mujoco_objects.sim->Load(mujoco_objects.m.get(), mujoco_objects.d.get(), model_path.c_str());

  {
    const std::unique_lock<std::recursive_mutex> lock(mujoco_objects.sim->mtx);
    mj_forward(mujoco_objects.m.get(), mujoco_objects.d.get());
    simulator.set_ctrl_noise(std::vector<double>(mujoco_objects.m->nu, 0));
  }

  return true;
}

double Mujoco::TimeApi::get_sim_time() const { return mujoco_objects.d->time; }

double Mujoco::TimeApi::get_slowdown_factor() const
{
  return static_cast<double>(100 / mujoco_objects.sim->percentRealTime[mujoco_objects.sim->real_time_index]);// NOLINT
}

bool Mujoco::TimeApi::is_simulation_speed_changed() const { return mujoco_objects.sim->speed_changed; }

void Mujoco::TimeApi::mark_simulation_speed_as_unchanged() { mujoco_objects.sim->speed_changed = false; }

void Mujoco::TimeApi::mark_simulation_speed_as_changed() { mujoco_objects.sim->speed_changed = true; }

bool Mujoco::TimeApi::is_cpu_and_sim_time_out_of_sync(TimePoint const &cpu_iteration_start,
  double sim_iteration_start) const
{
  auto const elapsed_cpu = cpu_iteration_start - cpu_sync_time;
  double const elapsed_sim = sim_iteration_start - sim_sync_time;
  double const slowdown = get_slowdown_factor();

  bool const misaligned = fabs(Seconds(elapsed_cpu).count() / slowdown - elapsed_sim) > sync_misalign_threshold;

  return elapsed_sim < 0 || elapsed_cpu.count() < 0 || cpu_sync_time.time_since_epoch().count() == 0 || misaligned
         || is_simulation_speed_changed();
}

bool Mujoco::TimeApi::is_sim_behind_cpu() const
{
  return Seconds((get_sim_time() - sim_sync_time) * get_slowdown_factor()) < Clock::now() - cpu_sync_time;
}

void Mujoco::TimeApi::synchronize_time(TimePoint const &cpu_iteration_start, double sim_iteration_start)
{
  cpu_sync_time = cpu_iteration_start;
  sim_sync_time = sim_iteration_start;
  mark_simulation_speed_as_unchanged();
}

bool Mujoco::TimeApi::is_sim_within_refresh_time(TimePoint const &cpu_iteration_start, int refresh_rate) const
{
  double const refresh_time = sim_refresh_fraction / refresh_rate;
  return Clock::now() - cpu_iteration_start < Seconds(refresh_time);
}

void Mujoco::TimeApi::measure_slowdown_factor(TimePoint const &cpu_iteration_start, double sim_iteration_start)
{
  double const elapsed_cpu = std::chrono::duration<double>((cpu_iteration_start - cpu_sync_time)).count();
  double const elapsed_sim = sim_iteration_start - sim_sync_time;

  // TODO: handle near-zero value here
  if (elapsed_sim == 0) { return; }

  mujoco_objects.sim->measured_slowdown = static_cast<float>(elapsed_cpu / elapsed_sim);
}

VectorX Mujoco::State::get_joint_positions() const
{
  return VectorX{ Eigen::Map<VectorX>(mujoco_objects.d->qpos, mujoco_objects.m->nq) };
}

VectorX Mujoco::State::get_joint_velocities() const
{
  return VectorX{ Eigen::Map<Eigen::VectorXd>(mujoco_objects.d->qvel, mujoco_objects.m->nv) };
}

VectorX Mujoco::State::get_joint_accelerations() const
{
  return VectorX{ Eigen::Map<Eigen::VectorXd>(mujoco_objects.d->qacc, mujoco_objects.m->nv) };
}

Vector3 Mujoco::State::get_mocap_position(int mocap_id) const
{
  return Vector3{ Eigen::Map<Eigen::VectorXd>(&mujoco_objects.d->mocap_pos[mocap_id], 3) };
}

SO3 Mujoco::State::get_mocap_orientation(int mocap_id) const
{
  Eigen::Map<Eigen::VectorXd> target_orientation_map(&mujoco_objects.d->mocap_quat[mocap_id], 4);
  legged_ctrl::Quaternion const target_quat{ // NOLINT
    target_orientation_map[0],
    target_orientation_map[1],
    target_orientation_map[2],
    target_orientation_map[3]
  };

  return target_quat.matrix();
}

bool Mujoco::Gui::is_simulation_paused() const { return mujoco_objects.sim->run == 0; }

bool Mujoco::Gui::is_exit_requested() const { return mujoco_objects.sim->exitrequest.load() != 0; }

void Mujoco::Gui::enter_render_loop() { mujoco_objects.sim->RenderLoop(); }

int Mujoco::Gui::refresh_rate() const { return mujoco_objects.sim->refresh_rate; }

void Mujoco::Gui::save_current_state_to_history() { mujoco_objects.sim->AddToHistory(); }

bool Mujoco::Gui::is_ctrl_noise_enabled() const { return mujoco_objects.sim->ctrl_noise_std != 0.; }

void Mujoco::Gui::pause_simulation() { mujoco_objects.sim->run = 0; }

void Mujoco::Simulator::step_simulation() { mj_step(mujoco_objects.m.get(), mujoco_objects.d.get()); }

void Mujoco::Simulator::forward() { mj_forward(mujoco_objects.m.get(), mujoco_objects.d.get()); }

void Mujoco::Simulator::apply_control_torques(VectorX const &tau, std::vector<int> const &actuator_ids)
{
  std::for_each(actuator_ids.begin(), actuator_ids.end(), [&](int const id) { mujoco_objects.d->ctrl[id] = tau(id); });
}

void Mujoco::Simulator::set_ctrl_noise(std::vector<double> noise) { mujoco_objects.ctrlnoise = std::move(noise); }

void Mujoco::Simulator::init_ctrl_noise_with_stddev()
{
  assert(mujoco_objects.ctrlnoise.size() == mujoco_objects.m->nu);
  mjtNum const rate = mju_exp(-mujoco_objects.m->opt.timestep / mju_max(mujoco_objects.sim->ctrl_noise_rate, mjMINVAL));
  mjtNum const scale = mujoco_objects.sim->ctrl_noise_std * mju_sqrt(1 - rate * rate);

  for (int i = 0; i < mujoco_objects.m->nu; ++i) {
    // update noise
    mujoco_objects.ctrlnoise[i] = rate * mujoco_objects.ctrlnoise[i] + scale * mju_standardNormal(nullptr);
    // apply noise
    mujoco_objects.d->ctrl[i] = mujoco_objects.ctrlnoise[i];
  }
}

namespace {
  int get_id(const char *target, int const *offsets, char const *names, int n)
  {
    for (int idx = 0; idx < n; ++idx) {
      const char *name = &names[offsets[idx]];
      if (strcmp(name, target) == 0) { return idx; }
    }

    return -1;
  }
}// namespace

void Mujoco::State::set_state_from_keyframe(const std::string &keyframe_name)
{
  int const key_id =
    get_id(keyframe_name.c_str(), mujoco_objects.m->name_keyadr, mujoco_objects.m->names, mujoco_objects.m->nkey);
  mj_resetDataKeyframe(mujoco_objects.m.get(), mujoco_objects.d.get(), key_id);
}

VectorX Mujoco::State::get_state_from_keyframe(const std::string &keyframe_name) const
{
  int const key_id =
    get_id(keyframe_name.c_str(), mujoco_objects.m->name_keyadr, mujoco_objects.m->names, mujoco_objects.m->nkey);

  return VectorX{ Eigen::Map<VectorX>(&mujoco_objects.m->key_qpos[key_id], mujoco_objects.m->nq) };
}

int Mujoco::Scene::get_mocap_id(const std::string &name) const
{
  int const mocap_body_id =
    get_id(name.c_str(), mujoco_objects.m->name_bodyadr, mujoco_objects.m->names, mujoco_objects.m->nbody);
  int const mocap_id = mujoco_objects.m->body_mocapid[mocap_body_id];

  return mocap_id;
}

int Mujoco::Scene::get_site_id(const std::string &name) const
{
  return get_id(name.c_str(), mujoco_objects.m->name_siteadr, mujoco_objects.m->names, mujoco_objects.m->nsite);
}

int Mujoco::Scene::get_actuator_id(const std::string &name) const
{
  return get_id(name.data(), mujoco_objects.m->name_actuatoradr, mujoco_objects.m->names, mujoco_objects.m->nu);
}

std::vector<int> Mujoco::Scene::get_actuator_ids(std::vector<std::string> const &joint_names) const
{
  std::vector<int> actuator_ids(joint_names.size());
  std::transform(joint_names.begin(), joint_names.end(), actuator_ids.begin(), [this](std::string const &name) {
    return get_actuator_id(name);
  });
  return actuator_ids;
}
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic, readability-make-member-function-const)

}// namespace task_space_control
