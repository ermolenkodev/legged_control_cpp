#include "mujoco_api.hpp"

namespace task_space_control {

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

mjModel *LoadModel(const char *file, mj::Simulate &sim)
{

  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) { return nullptr; }

  // load and compile
  const int kErrorLength = 1024;
  char loadError[kErrorLength] = "";
  mjModel *mnew = 0;
  if (mju::strlen_arr(filename) > 4
      && !std::strncmp(
        filename + mju::strlen_arr(filename) - 4, ".mjb", mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) { mju::strcpy_arr(loadError, "could not load binary model"); }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
    // remove trailing newline character from loadError
    if (loadError[0]) {
      std::size_t error_length = mju::strlen_arr(loadError);
      if (loadError[error_length - 1] == '\n') { loadError[error_length - 1] = '\0'; }
    }
  }

  mju::strcpy_arr(sim.load_error, loadError);

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  return mnew;
}

bool Mujoco::load_model(const std::string &model_path)
{
  mujoco_objects.sim->LoadMessage(model_path.c_str());
  mujoco_objects.m = { LoadModel(model_path.c_str(), *mujoco_objects.sim), mj_deleteModel };
  if (mujoco_objects.m) {
    // lock the sim mutex
    const std::unique_lock<std::recursive_mutex> lock(mujoco_objects.sim->mtx);

    mujoco_objects.d = { mj_makeData(mujoco_objects.m.get()), mj_deleteData };
  }
  if (mujoco_objects.d) {
    mujoco_objects.sim->Load(mujoco_objects.m.get(), mujoco_objects.d.get(), model_path.c_str());

    // lock the sim mutex
    const std::unique_lock<std::recursive_mutex> lock(mujoco_objects.sim->mtx);

    mj_forward(mujoco_objects.m.get(), mujoco_objects.d.get());

    simulator.set_ctrl_noise(std::vector<int>(mujoco_objects.m->nu, 0));
  } else {
    mujoco_objects.sim->LoadMessageClear();
    return false;
  }

  return true;
}

double Mujoco::TimeApi::get_sim_time() const { return mujoco_objects.d->time; }

double Mujoco::TimeApi::get_slowdown_factor() const
{
  return static_cast<double>(100 / mujoco_objects.sim->percentRealTime[mujoco_objects.sim->real_time_index]);
}

bool Mujoco::TimeApi::is_simulation_speed_changed() const { return mujoco_objects.sim->speed_changed; }

void Mujoco::TimeApi::mark_simulation_speed_as_unchanged() { mujoco_objects.sim->speed_changed = false; }

void Mujoco::TimeApi::mark_simulation_speed_as_changed() { mujoco_objects.sim->speed_changed = true; }

bool Mujoco::TimeApi::is_cpu_and_sim_time_out_of_sync(TimePoint const &cpu_iteration_start, double sim_iteration_start) const
{
  const auto elapsed_cpu = cpu_iteration_start - cpu_sync_time;
  double elapsed_sim = sim_iteration_start - sim_sync_time;
  double slowdown = get_slowdown_factor();

  bool misaligned = fabs(Seconds(elapsed_cpu).count() / slowdown - elapsed_sim) > sync_misalign_threshold;

  return elapsed_sim < 0 || elapsed_cpu.count() < 0 || cpu_sync_time.time_since_epoch().count() == 0 || misaligned
         || is_simulation_speed_changed();
}

bool Mujoco::TimeApi::is_sim_behind_cpu() const
{
  return Seconds((get_sim_time() - sim_sync_time)*get_slowdown_factor()) < Clock::now() - cpu_sync_time;
}

void Mujoco::TimeApi::synchronize_time(TimePoint const &cpu_iteration_start, double sim_iteration_start)
{
  cpu_sync_time = cpu_iteration_start;
  sim_sync_time = sim_iteration_start;
  mark_simulation_speed_as_unchanged();
}

bool Mujoco::TimeApi::is_sim_within_refresh_time(TimePoint const &cpu_iteration_start, int refresh_rate) const
{
  double refresh_time = sim_refresh_fraction / refresh_rate;
  return Clock::now() - cpu_iteration_start < Seconds(refresh_time);
}

void Mujoco::TimeApi::measure_slowdown_factor(TimePoint const &cpu_iteration_start, double sim_iteration_start)
{
  double elapsed_cpu = std::chrono::duration<double>((cpu_iteration_start - cpu_sync_time)).count();
  double elapsed_sim = sim_iteration_start - sim_sync_time;

  // TODO: handle near-zero value here
  if (elapsed_sim == 0) { return; }

  double slowdown = elapsed_cpu / elapsed_sim;

  mujoco_objects.sim->measured_slowdown = slowdown;
}

VectorX Mujoco::State::get_joint_positions() const
{
  return VectorX{ Eigen::Map<VectorX>(mujoco_objects.d->qpos, mujoco_objects.m->nq) };
}

VectorX Mujoco::State::get_joint_velocities() const
{
  return VectorX{ Eigen::Map<Eigen::VectorXd>(mujoco_objects.d->qvel, mujoco_objects.m->nv) };
}

Vector3 Mujoco::State::get_mocap_position(int mocap_id) const
{
  return Vector3{ Eigen::Map<Eigen::VectorXd>(&mujoco_objects.d->mocap_pos[mocap_id], 3) };
}

SO3 Mujoco::State::get_mocap_orientation(int mocap_id) const
{
  Eigen::Map<Eigen::VectorXd> target_orientation_map(&mujoco_objects.d->mocap_quat[mocap_id], 4);
  legged_ctrl::Quaternion target_quat{
    target_orientation_map[0], target_orientation_map[1], target_orientation_map[2], target_orientation_map[3]
  };

  return target_quat.matrix();
}

bool Mujoco::Gui::is_simulation_paused() const { return mujoco_objects.sim->run == 0; }

bool Mujoco::Gui::is_exit_requested() const { return mujoco_objects.sim->exitrequest.load(); }

void Mujoco::Gui::enter_render_loop() { mujoco_objects.sim->RenderLoop(); }

int Mujoco::Gui::refresh_rate() const { return mujoco_objects.sim->refresh_rate; }

void Mujoco::Gui::save_current_state_to_history() { mujoco_objects.sim->AddToHistory(); }

bool Mujoco::Gui::is_ctrl_noise_enabled() const { return mujoco_objects.sim->ctrl_noise_std != 0.; }

void Mujoco::Simulator::step_simulation() { mj_step(mujoco_objects.m.get(), mujoco_objects.d.get()); }

void Mujoco::Simulator::forward() { mj_forward(mujoco_objects.m.get(), mujoco_objects.d.get()); }

void Mujoco::Simulator::apply_control_torques(VectorX const &tau, std::vector<int> const &actuator_ids)
{
  std::for_each(actuator_ids.begin(), actuator_ids.end(), [&](int const id) { mujoco_objects.d->ctrl[id] = tau(id); });
}

void Mujoco::Simulator::set_ctrl_noise(std::vector<int> noise)
{
  mujoco_objects.ctrlnoise = std::move(noise);
}

void Mujoco::Simulator::init_ctrl_noise_with_stddev()
{
  assert(mujoco_objects.ctrlnoise.size() == mujoco_objects.m->nu);
  mjtNum rate = mju_exp(-mujoco_objects.m->opt.timestep / mju_max(mujoco_objects.sim->ctrl_noise_rate, mjMINVAL));
  mjtNum scale = mujoco_objects.sim->ctrl_noise_std * mju_sqrt(1-rate*rate);

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

void Mujoco::Scene::set_state_from_keyframe(const std::string &keyframe_name)
{
  int const key_id =
    get_id(keyframe_name.c_str(), mujoco_objects.m->name_keyadr, mujoco_objects.m->names, mujoco_objects.m->nkey);
  mj_resetDataKeyframe(mujoco_objects.m.get(), mujoco_objects.d.get(), key_id);
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

std::vector<int> Mujoco::Scene::get_actuator_ids(std::vector<std::string> joint_names) const
{
  std::vector<int> actuator_ids;
  for (const auto &name : joint_names) { actuator_ids.push_back(get_actuator_id(name)); }

  return actuator_ids;
}

}// namespace task_space_control
