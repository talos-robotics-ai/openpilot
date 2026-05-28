# policypilot — Architecture

This document explains the design decisions behind policypilot. For the
high-level pitch, see the top-level [README](../README.md).

## 1. Two runtimes, one bus

```
┌──────────────────────────────────────────┐        ┌──────────────────────────────────────────┐
│  ROS 2 runtime  (ros2_ws + ROS python)   │        │  policy_runtime  (policypilot-runtime conda env)  │
│                                          │        │                                          │
│   robot_state, loco_client, ...          │        │   run_pipeline.py                        │
│   policy_manager  ────────► subprocess ──┼───────▶│     ↳ AMO policy + UnitreeCtrl           │
│                                          │        │                                          │
└─────┬────────────────────────────────────┘        └─────┬────────────────────────────────────┘
      │                                                  │
      │       Unitree DDS bus  (rt/lowstate, rt/arm_sdk, dex3 topics, loco RPC)
      └──────────────────────────────────────────────────┘
```

Both processes are DDS participants on the same network interface. They never
share a Python interpreter. The only ROS-side handle on the policy runtime is
the subprocess `Popen` object owned by `policy_manager`.

### Why subprocess, not a ROS node?

We considered three options for hosting the RL policy:

| Option | Why we rejected it |
| --- | --- |
| Make RoboJuDo a ROS node | Would force the ROS Python to import torch / mujoco / RoboJuDo's whole stack — fragile, hard to pin |
| Run RoboJuDo as a ROS *component* with cross-process IPC over services | Adds a service layer that buys us nothing; the policy already talks DDS directly to the robot |
| **Subprocess managed by a ROS supervisor** ← chosen | The policy ships as a normal CLI tool, the env is fully isolated, the supervisor only watches the lifecycle |

Bonus: the same `run_pipeline.py` can be launched by hand for debugging,
without ROS in the loop.

### Why one supervisor node?

We deliberately keep `policy_manager` as a single node that owns the
subprocess. Splitting "start", "stop", "monitor" into separate nodes would
mean each one has to coordinate over ROS topics about who actually owns the
process handle. One supervisor avoids that bookkeeping.

## 2. The ROS package layout

The ROS Python package is grouped by **what the code controls**:

| Subpackage | Owns | Talks to |
| --- | --- | --- |
| `state/` | `rt/lowstate` reader → `/joint_states`, IMU, motor telemetry, TF | DDS (read) |
| `locomotion/` | Unitree loco RPC client + planners (nav2point, dijkstra) | Unitree loco RPC (write), DDS (read) |
| `manipulation/` | Arm DDS writer + DEX3 hand controller + RViz markers | DDS (write) |
| `policy/` | Supervises one RoboJuDo subprocess | `subprocess.Popen`, ROS topics for lifecycle |
| `dashboard/` | PyQt6 operator panel (AMO WALK / START / hands / E-STOP) | ROS topics only — owns no state |
| `utils/` | IK, joint maps, config helpers (no ROS in itself) | nothing |
| `tools/` | Standalone debug scripts | varies |

We removed `teleoperation/` entirely. The directory existed in `g1pilot`
purely because that project mixed *VR/arm teleop bridges* (wrist ZMQ,
hand glove) with the *policy launcher* and the *operator dashboard*. Here:

- the launcher lives in `policy/`,
- the operator dashboard lives in `dashboard/`,
- the VR/arm teleop bridges simply do not exist.

The dashboard publishes lifecycle topics only — it owns no robot state —
which keeps the door open for replacing it with a web UI, a CLI, or
another front-end without touching the rest of the stack.

### Renames from g1pilot

| g1pilot | policypilot | Reason |
| --- | --- | --- |
| `teleoperation/low_level_manager.py` | `policy/policy_manager.py` | The node never was a "low level" thing — it's the policy supervisor |
| `navigation/` | `locomotion/` | The bulk of the code is loco-RPC, not nav-stack |
| `/g1pilot/low_level/*` topics | `/policypilot/policy/*` | Matches the new node naming |
| `G1PILOT_*` env vars | `POLICYPILOT_*` | Cosmetic but consistent |

## 3. Configuration: one yaml, one schema

`config/config.yaml` is the single source of truth for *defaults*. Every
field becomes a ROS parameter on the relevant node. Operators can override
any parameter on the command line at launch time; nodes never re-read the
yaml at runtime.

Sections:

- `general:` — values that several launchers share (network interface, sim
  rate, frame names).
- `nodes:` — booleans that toggle launchers inside `bringup_launcher`.
- `policy:` — defaults for `policy_manager` (= what RoboJuDo gets on its CLI).
- `inverse_kinematics:`, `home:`, `workspace:`, `robot:`, `state:`, `dex3:`,
  `manipulation:`, `navigation:` — node-specific groups; only the owning
  node reads its own section.

Rationale: keeping defaults declarative in yaml means launchers stay short
(they just pass yaml values through), and operators have one file to grep
when they need to know "what's the default X". The full reference is in
[CONFIGURATION.md](CONFIGURATION.md).

## 4. Launch composition

```
bringup_launcher
 ├─ livox_launcher           (LiDAR driver)
 ├─ locomotion_launcher      (loco_client + planners)
 ├─ robot_state_launcher     (state + TF + RViz)
 ├─ policy_launcher          (policy_manager)
 ├─ manipulation_launcher    (arm_controller + dx3 + markers)
 └─ dashboard_launcher       (control_panel — PyQt operator GUI)
```

`bringup_launcher` is a composition launcher. Every sub-launcher is gated
by an `IfCondition` driven by `nodes.*_enabled` in `config.yaml`, so any
subsystem can be turned off without editing files:

```bash
ros2 launch policypilot bringup_launcher.launch.py start_policy:=false
```

`mola_launcher` is intentionally **not** in `bringup_launcher`. MOLA needs a
hot LiDAR feed before it can initialize, so the operational pattern is
"start bringup, wait for LiDAR, then start MOLA in a second terminal."

## 5. What is *not* in this package

These are intentional omissions, not TODOs:

- **VR / arm teleop** — no wrist-pose feeds, no `TeleVision`, no hand-glove
  ZMQ bridges. The `g1_amo_arm_teleop_real` pipeline is still present in the
  bundled RoboJuDo for advanced users, but the ROS side does not feed it.
- **Joystick mux for the AMO policy** — the AMO controller reads the
  Unitree handheld remote directly from the DDS bus, so a ROS-side
  joystick → policy bridge would be redundant. The dashboard does ship for
  high-level lifecycle (start/stop/balance/arms).
- **Training infrastructure** — RoboJuDo's training side is not the focus
  here. policypilot is a *runtime* package; train policies upstream and
  drop checkpoints into `policy_runtime/`.

## 6. When you should add something here

Use the following decision tree:

```
Does the new code talk ROS?
├─ no  → it probably belongs in policy_runtime/ as part of RoboJuDo
└─ yes → which boundary?
        ├─ reads robot state          → state/
        ├─ commands locomotion        → locomotion/
        ├─ commands arms or hands     → manipulation/
        ├─ supervises a subprocess /
        │  RL policy                  → policy/
        └─ purely numeric / config /
           IK helpers, no ROS         → utils/
```

If a new directory is needed, add a launcher for it and wire that launcher
into `bringup_launcher` with a `nodes.X_enabled` flag.
