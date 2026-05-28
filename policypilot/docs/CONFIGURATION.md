# Configuration Reference

All defaults live in [`config/config.yaml`](../config/config.yaml). Every
field is mapped onto a ROS parameter on the relevant node; nothing is
re-read from yaml at runtime. To override at launch time:

```bash
ros2 launch policypilot bringup_launcher.launch.py interface:=eth0 start_policy:=false
ros2 run policypilot policy_manager --ros-args -p config_name:=g1_amo_arm_teleop_real
```

To use a non-default yaml file:

```bash
export POLICYPILOT_CONFIG=/etc/policypilot/site_overrides.yaml
```

---

## `general:`

| Key | Default | Read by | Meaning |
| --- | --- | --- | --- |
| `use_sim_time` | `false` | launchers | Use Gazebo clock |
| `use_robot` | `true` | most nodes | Talk to the real robot (vs simulation/mock) |
| `publish_joint_states` | `true` | `robot_state` | Publish `/joint_states` |
| `interface` | `enxc8a362edcebb` | most nodes | Network interface for Unitree DDS / loco RPC |
| `sim_rate_hz` | `50.0` | `robot_state` | Publish rate when `use_robot=false` |
| `arm_controlled` | `both` | `arm_controller`, `dx3` | `left`, `right`, or `both` |
| `enable_arm_ui` | `true` | `loco_client` | Whether to wire up arm UI helpers |
| `use_waist` | `false` | `arm_controller` | Include waist DoFs in IK |
| `frame_link` | `pelvis` | `arm_controller` | Root frame for IK |
| `use_mola`, `use_livox` | `true` | docs only | Operator hint; actual on/off via `nodes.*_enabled` |

## `nodes:`

Booleans consumed by `bringup_launcher` to gate sub-launchers.

| Key | Default | Subsystem |
| --- | --- | --- |
| `livox_launcher_enabled` | `true` | LiDAR driver |
| `mola_launcher_enabled` | `true` | MOLA odometry (note: not in bringup; flag is informational) |
| `locomotion_launcher_enabled` | `true` | `loco_client` + planners |
| `robot_state_launcher_enabled` | `true` | State + TF + RViz |
| `manipulation_launcher_enabled` | `true` | Arm + hand |
| `policy_launcher_enabled` | `true` | `policy_manager` |
| `dashboard_launcher_enabled` | `true` | PyQt `control_panel` |

## `policy:`

Defaults for `policy_manager`. These build the subprocess CLI; see
[ROBOJUDO_INTEGRATION.md §3.3](ROBOJUDO_INTEGRATION.md#33-the-environment-overrides)
for what each one does.

| Key | Default | Meaning |
| --- | --- | --- |
| `python_executable` | `/opt/policypilot-runtime/bin/python` | Interpreter for the policy subprocess |
| `conda_prefix` | `/opt/policypilot-runtime` | Conda env root (sets `LD_LIBRARY_PATH`, `PATH`, `CONDA_PREFIX`) |
| `mplconfigdir` | `/tmp/matplotlib` | matplotlib cache dir for the subprocess |
| `policy_runtime` | *auto-detect* | Path to the bundled `policy_runtime/`. Only set if moved out-of-tree |
| `runner_script` | *auto-detect* | `policy_runtime/scripts/run_pipeline.py` |
| `working_directory` | *auto-detect* | CWD for the subprocess (defaults to `policy_runtime/`'s parent) |
| `config_name` | `g1_amo_real` | RoboJuDo `cfg_registry` entry to spawn |
| `interface` | `enxc8a362edcebb` | NIC for Unitree DDS inside the subprocess (`--iface`) |
| `record` | `false` | Pass `--record` to enable episode logging |
| `img_server_ip` | `192.168.123.164` | Image server (`--img-server-ip`) |
| `task_dir` | `/data/policypilot_runs` | Where episode logs land (`--task-dir`) |
| `task_name` | `g1_amo_run` | (`--task-name`) |
| `task_goal` / `task_desc` / `task_steps` | `""` | Free-text run metadata |

Two pipelines worth knowing about (both in
[`policy_runtime/robojudo/config/g1/g1_cfg.py`](../policy_runtime/robojudo/config/g1/g1_cfg.py)):

| `config_name` | What it does | Requires |
| --- | --- | --- |
| `g1_amo_real` | AMO balance, no arm overrides | nothing extra |
| `g1_amo_arm_teleop_real` | AMO balance + arm teleop | wrist ZMQ feed on `127.0.0.1:55556` (not provided by policypilot) |

## `inverse_kinematics:`

Tuning knobs for `arm_controller`'s Pinocchio IK loop.

| Key | Default | Meaning |
| --- | --- | --- |
| `alpha` | `0.2` | IK damping |
| `max_dq_step` | `0.05` | Per-tick joint delta limit (rad) |
| `velocity_limit` | `5.0` | Max joint velocity (rad/s) |
| `kp_high`, `kd_high`, `kp_low`, `kd_low`, `kp_wrist`, `kd_wrist` | varies | PD gains per joint class |

## `home:`

Per-motor target positions for the "home" arm pose. Two subkeys
(`left:`, `right:`), each with `motor_1`..`motor_7`. Triggered by
publishing `true` on `/policypilot/arms/home`.

## `workspace:`

Eight 3-D corner points defining each arm's allowed end-effector volume.
Published as RViz markers by `arm_controller`. Frame is `frame:` (default
`pelvis`).

## `robot:`

Names of topics and TF frames used across the manipulation stack.

| Key | Default |
| --- | --- |
| `model` | `29dof` |
| `left_eff_topic` | `/policypilot/left_hand_goal` |
| `right_eff_topic` | `/policypilot/right_hand_goal` |
| `left_tf_frame` | `left_hand_point_contact` |
| `right_tf_frame` | `right_hand_point_contact` |
| `start_topic` | `/policypilot/start` |
| `balance_topic` | `/policypilot/balance` |
| `emergency_stop_topic` | `/policypilot/emergency_stop` |

## `state:`

| Key | Default |
| --- | --- |
| `imu_topic` | `/policypilot/imu` |
| `imu_link` | `imu_link` |
| `motors_topic` | `/policypilot/motor_state` |
| `odom_topic` | `/policypilot/odom` |
| `voice_topic` | `/policypilot/voice` |

## `dex3:`

DEX3 hand configuration: open/close DoF presets, DDS topic names, and
ROS-side action topics.

| Key | Meaning |
| --- | --- |
| `total_dofs` | 7 |
| `{left,right}_gripper.{open,close}_values` | Per-finger target arrays |
| `send_commands` | If `false`, run dry (publish state only) |
| `{left,right}_cmd_topic` / `{left,right}_state_topic` | DDS topics |
| `{left,right}_hand_topic` | ROS action topic for `dx3_controller` |

## `manipulation:`

Point-goal topics for the IK pipeline.

## `navigation:`

Joystick type, map and odom topics, goal/path topics, and per-axis velocity
limits used by `nav2point` and `dijkstra_planner`.

| Key | Default |
| --- | --- |
| `joystick_type` | `Wireless Controller` |
| `map_topic` | `/map` |
| `odom_topic` | `/lidar_odometry/pose_fixed` |
| `vx_limit`, `vy_limit`, `wz_limit` | `0.6, 0.6, 0.5` |
| `goal_topic`, `path_topic`, `goal_marker_topic`, ... | `/policypilot/...` |

---

## Environment variables that override yaml

| Var | Read by | Behavior |
| --- | --- | --- |
| `POLICYPILOT_CONFIG` | every node + every launcher | Path to an alternate `config.yaml` |
| `POLICYPILOT_INTERFACE` | every launcher | Overrides `general.interface` for the bringup |
| `POLICYPILOT_ROOT` | `policy_manager` | Where to look for `policy_runtime/` |
| `POLICYPILOT_IK_NO_COLLISION` | `utils/ik_solver.py` | Disable Pinocchio collision env if set to `1`/`true`/`yes` |
