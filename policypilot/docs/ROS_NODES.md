# ROS Nodes Reference

One row per node. Topic names are absolute. All parameters can be
overridden at launch time or by editing `config/config.yaml`.

> Note: This is a directory of the **interface surface**, not an exhaustive
> spec. For implementation detail, read the node source.

---

## policy/

### `policy_manager`

Supervises a single RoboJuDo pipeline subprocess. See
[ROBOJUDO_INTEGRATION.md](ROBOJUDO_INTEGRATION.md) for the deep dive.

| Direction | Topic | Type | Purpose |
| --- | --- | --- | --- |
| sub | `/policypilot/policy/start` | `std_msgs/Bool` | `true` → spawn subprocess |
| sub | `/policypilot/policy/stop` | `std_msgs/Bool` | `true` → SIGTERM (5 s) → SIGKILL |
| sub | `/policypilot/emergency_stop` | `std_msgs/Bool` | `true` → immediate stop |
| pub | `/policypilot/policy/running` | `std_msgs/Bool` | Subprocess alive |
| pub | `/policypilot/policy/status` | `std_msgs/String` | Free-form status |

Key parameters (defaults from `policy:` in `config.yaml`):
`python_executable`, `conda_prefix`, `policy_runtime`, `runner_script`,
`config_name`, `interface`, `record`, `task_dir`, `img_server_ip`.

---

## state/

### `robot_state`

Reads `rt/lowstate` from the Unitree DDS bus and republishes structured ROS
messages.

| Direction | Topic | Type | Purpose |
| --- | --- | --- | --- |
| pub | `/joint_states` | `sensor_msgs/JointState` | Joint positions (TF + RViz) |
| pub | `/policypilot/imu` | `sensor_msgs/Imu` | Pelvis IMU |
| pub | `/policypilot/motor_state` | `unitree_hg/MotorStateList` | Per-motor telemetry |

Parameters: `use_robot`, `interface`, `publish_joint_states`, `sim_rate_hz`.

### `lights` and `voice`

Small helper nodes for status LEDs and TTS. Read their source for the
topic surface — they are not on the critical path.

---

## locomotion/

### `loco_client`

Bridges ROS commands to the Unitree loco RPC server. This is the path used
to send the robot into BalanceStand, Damp, walking, etc.

| Direction | Topic | Type | Purpose |
| --- | --- | --- | --- |
| sub | `/policypilot/emergency_stop` | `Bool` | Damp |
| sub | `/policypilot/start` | `Bool` | Loco start sequence |
| sub | `/policypilot/start_balancing` | `Bool` | Unitree BalanceStand |
| sub | `/policypilot/joy` | `sensor_msgs/Joy` | Velocity commands from joystick / planner |
| pub | `/policypilot/arms/enabled` | `Bool` | Whether arms accept ROS commands |
| pub | `/policypilot/dx3/hand_action/{left,right}` | `String` | Gripper open/close requests |
| pub | `/policypilot/arms/home` | `Bool` | Send arms to home pose |

> **Note** there are two "balance" concepts in this package:
> - **Unitree BalanceStand** (this node, `/policypilot/start_balancing`).
> - **RoboJuDo AMO balance** (the policy_manager, `/policypilot/policy/start`).
> They are different controllers; do not start both at once.

Parameters: `interface`, `use_robot`, `arm_controlled`, `enable_arm_ui`,
`ik_*`, `arm_velocity_limit`.

### `nav2point`

Converts a `nav_msgs/Path` to `sensor_msgs/Joy` body-velocity commands via
a simple position/heading P controller. Feeds the loco_client.

| Direction | Topic | Type |
| --- | --- | --- |
| sub | `/lidar_odometry/pose_fixed` | `nav_msgs/Odometry` |
| sub | `/policypilot/auto_enable` | `Bool` |
| sub | `/policypilot/path` | `nav_msgs/Path` |
| pub | `/policypilot/auto_joy` | `sensor_msgs/Joy` |
| pub | `/policypilot/{waypoint,goal}_marker` | `visualization_msgs/Marker` |

### `dijkstra_planner`

Grid Dijkstra over an `OccupancyGrid`, producing a smoothed `nav_msgs/Path`
that `nav2point` follows.

| Direction | Topic | Type |
| --- | --- | --- |
| sub | `/map` | `OccupancyGrid` |
| sub | `/lidar_odometry/pose_fixed` | `nav_msgs/Odometry` |
| sub | `/policypilot/goal` | `geometry_msgs/PoseStamped` |
| pub | `/policypilot/path` | `nav_msgs/Path` |

### `fix_mola_odometry`

Re-publishes MOLA odometry on `/lidar_odometry/pose_fixed` in the frame
the planner expects.

### `create_map`, `loco_rpc_test`, `sport_service_test`

Tools, not part of the runtime stack. Read their `--help`.

---

## manipulation/

### `arm_controller`

Pinocchio-based IK + DDS writer for the G1 arms. Accepts pose goals on
`/policypilot/hand_goal/{left,right}` and writes joint targets to
`rt/arm_sdk`.

| Direction | Topic | Type |
| --- | --- | --- |
| sub | `/policypilot/hand_goal/{left,right}` | `geometry_msgs/PoseStamped` |
| sub | `/policypilot/arms/enabled` | `Bool` |
| sub | `/policypilot/arms/home` | `Bool` |
| pub | `/policypilot/workspace/{left,right}` | `visualization_msgs/Marker` |

Parameters (selection): `interface`, `use_robot`, `arm_velocity_limit`,
`ik_world_frame`, `ik_use_waist`, `ik_alpha`, `ik_max_dq_step`,
`ik_goal_filter_alpha`, `ik_orientation_mode`, `ik_max_ori_step_rad`,
`ee_auto_calibrate`, `ee_offset_*`, `auto_reissue_goals`, `goal_pos_tol`,
`goal_ori_tol_deg`.

### `dx3_controller` (`dx3_hand.py`)

DEX3 hand controller. Subscribes to hand-action strings and publishes per-
finger motor states.

| Direction | Topic | Type |
| --- | --- | --- |
| pub | `policypilot/dx3/{left,right}/motor_state` | `unitree_hg/MotorStateList` |
| sub | `policypilot/dx3/hand_action/{left,right}` | `String` |

### `interactive_marker`

Spawns RViz interactive markers that publish `PoseStamped` goals on
`/policypilot/hand_goal/{left,right}`.

---

## dashboard/

### `control_panel`

PyQt6 operator window — a 5×5 grid of buttons that publish lifecycle ROS
topics. Owns no robot state itself.

| Button | Topic | Behavior |
| --- | --- | --- |
| `START` | `/policypilot/start` | Loco RPC start sequence (1 s flash) |
| `START BALANCING` | `/policypilot/start_balancing` | Unitree BalanceStand (1 s flash) |
| `START + BALANCE` | both above, ~700 ms apart | Convenience sequence |
| `AMO WALK` | `/policypilot/policy/start` | Spawns RoboJuDo `g1_amo_real` via `policy_manager` |
| `STOP POLICY` | `/policypilot/policy/stop` | Terminates the RoboJuDo subprocess |
| `HOMING ARMS` | `/policypilot/arms/home` | |
| `ENABLE MANIPULATION` | `/policypilot/arms/enabled` | Toggle |
| `OPEN/CLOSE {LEFT,RIGHT} HAND` | `/policypilot/dx3/hand_action/{left,right}` | String `"open"` / `"close"` |
| `EMERGENCY STOP` | `/policypilot/emergency_stop` + `/policypilot/policy/stop` | Hard stop everything |

The bottom status bar mirrors `/policypilot/policy/{running,status}` so the
operator can see whether the AMO subprocess is alive.

---

## utils/, tools/

Helper modules and standalone scripts. No ROS nodes here.

---

## Launch surface

| Launcher | Includes |
| --- | --- |
| `bringup_launcher.launch.py` | All sub-launchers, gated by `nodes.*_enabled` |
| `policy_launcher.launch.py` | `policy_manager` |
| `locomotion_launcher.launch.py` | `loco_client`, `nav2point`, `dijkstra_planner` |
| `manipulation_launcher.launch.py` | `arm_controller`, `dx3_controller`, `interactive_marker` |
| `dashboard_launcher.launch.py` | `control_panel` (PyQt) |
| `robot_state_launcher.launch.py` | `robot_state`, `mola_fixed`, static TFs, RViz2 |
| `livox_launcher.launch.py` | Livox LiDAR driver |
| `mola_launcher.launch.py` | MOLA odometry (run separately, after LiDAR is up) |
