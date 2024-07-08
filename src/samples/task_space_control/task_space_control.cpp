#include "gains.hpp"
#include "mujoco_api.hpp"

#define ASSETS_PATH CMAKE_ASSETS_PATH

using namespace task_space_control;

#define ENABLE_INVERSE_DYNAMICS 1
#define ENABLE_POSTURE_CONTROL 1

VectorX
  compute_control_torques(Mujoco const &mujoco, MultibodyModel const &model, int mocap_id, GainMatrices const &gains)
{
  auto q = mujoco.state.get_joint_positions();
  auto [wPee, wRee] = legged_ctrl::compute_end_effector_placement(model, q);

  auto wPd = mujoco.state.get_mocap_position(mocap_id);

  auto wRd = mujoco.state.get_mocap_orientation(mocap_id);

  auto eeRd = wRee.transpose() * wRd;
  auto [delta_theta, r_hat] = legged_ctrl::angle_axis_from_SO3(eeRd);

  auto eeO_error = delta_theta * r_hat;// expressed in the end-effector frame

  // we need to convert the orientation error to the world frame
  // because we will apply Jacobian to it and Jacobian is expressed in local frame
  // with the orientation aligned with the world frame
  auto wO_error = wRee * eeO_error;// expressed in the world frame

  auto J = legged_ctrl::compute_end_effector_frame_jacobian(model, q, legged_ctrl::ReferenceFrame::LOCAL_WORLD_ALIGNED);

  auto qd = mujoco.state.get_joint_velocities();
  auto twist = J * qd;
  auto v = twist.tail<3>();
  auto omega = twist.head<3>();

  auto [Kxp, Kxd, Kop, Kod] = gains;

  legged_ctrl::SpatialVector F;
  F.tail(3) = Kxp * (wPd - wPee) + Kxd * (-v);
  F.head(3) = Kop * wO_error + Kod * (-omega);

#if ENABLE_INVERSE_DYNAMICS
  auto [M, C] = legged_ctrl::crba(model, legged_ctrl::SystemConfiguration{ q, qd });
  auto lambda = (J * M.inverse() * J.transpose()).completeOrthogonalDecomposition().pseudoInverse();

  //  TODO: fix mu term computation
  //  auto Jdqd = legged_ctrl::compute_end_effector_classical_acceleration(model,
  //    legged_ctrl::SystemConfiguration{ q, qd, qdd }, legged_ctrl::ReferenceFrame::LOCAL_WORLD_ALIGNED);
  //  auto mu = JTpinv * C - lambda * Jdqd;

  F = lambda * F;
  VectorX tau = J.transpose() * F + C;

#if ENABLE_POSTURE_CONTROL
  auto JTpinv = J.transpose().completeOrthogonalDecomposition().pseudoInverse();
  MatrixX N = eye(model.n_bodies) - J.transpose() * JTpinv;
  VectorX desired_posture = mujoco.state.get_state_from_keyframe("home");
  tau += N * M * (100 * (desired_posture - q) + 10 * -qd);
#endif

  return tau;
#else
  auto g = legged_ctrl::compute_gravity_effect(model, legged_ctrl::SystemConfiguration{ q });

  return J.transpose() * F + g;
#endif
}

void simulation_and_control_loop(Mujoco &mujoco, std::string const &scene_path, MultibodyModel const &model)
{
  bool const ok = mujoco.load_model(scene_path);
  if (!ok) { throw std::runtime_error("Failed to load scene"); }

  mujoco.state.set_state_from_keyframe("home");

  int const mocap_id = mujoco.scene.get_mocap_id("target");
  std::vector<int> const actuator_ids =
    mujoco.scene.get_actuator_ids({ "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7" });

#if ENABLE_INVERSE_DYNAMICS
  GainMatrices const gains{ 300, 30, 100, 10 };
#else
  GainMatrices const gains{ 500, 50, 10, .1 };
#endif

  while (!mujoco.gui.is_exit_requested()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    {
      std::unique_lock<std::recursive_mutex> const lock(mujoco.mutex());

      if (mujoco.gui.is_simulation_paused()) {
        mujoco.simulator.forward();
        mujoco.time_api.mark_simulation_speed_as_changed();
        continue;
      }

      bool stepped = false;

      auto const cpu_iteration_start = Mujoco::Clock::now();
      double const sim_iteration_start = mujoco.time_api.get_sim_time();

      if (mujoco.gui.is_ctrl_noise_enabled()) { mujoco.simulator.init_ctrl_noise_with_stddev(); }

      // out-of-sync (for any reason): reset sync times, step
      if (mujoco.time_api.is_cpu_and_sim_time_out_of_sync(cpu_iteration_start, sim_iteration_start)) {
        mujoco.time_api.synchronize_time(cpu_iteration_start, sim_iteration_start);
        // run single step, let next iteration deal with timing
        mujoco.simulator.step_simulation();
        stepped = true;
      }
      // in-sync: step until ahead of cpu
      else {
        bool measured = false;
        double const prev_sim = mujoco.time_api.get_sim_time();

        // step while sim lags behind cpu and within refreshTime
        while (mujoco.time_api.is_sim_behind_cpu()
               and mujoco.time_api.is_sim_within_refresh_time(cpu_iteration_start, mujoco.gui.refresh_rate())) {
          // measure slowdown before first step
          if (!measured) {
            mujoco.time_api.measure_slowdown_factor(cpu_iteration_start, sim_iteration_start);
            measured = true;
          }

          VectorX const tau = compute_control_torques(mujoco, model, mocap_id, gains);

          mujoco.simulator.apply_control_torques(tau, actuator_ids);
          mujoco.simulator.step_simulation();
          stepped = true;

          // break if reset
          if (mujoco.time_api.get_sim_time() < prev_sim) { break; }
        }
      }

      // save current state to history buffer
      if (stepped) { mujoco.gui.save_current_state_to_history(); }
    }// release lock
  }
}

int main()
{
  auto mujoco = Mujoco();
  MultibodyModel const model = legged_ctrl::mjxml::parse_mujoco_xml(std::string(ASSETS_PATH) + "/scene/iiwa14.xml");

  std::thread simulation_thread(
    &simulation_and_control_loop, std::ref(mujoco), std::string(ASSETS_PATH) + "/scene/scene.xml", model);
  mujoco.gui.enter_render_loop();
  simulation_thread.join();

  return 0;
}
