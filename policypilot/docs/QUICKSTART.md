# Quickstart — Walking the G1 with AMO

This is the **beginner walkthrough**. End-to-end you will:

1. Build & start the Docker container.
2. Bring up policypilot inside it.
3. Click **AMO WALK** on the dashboard.
4. Drive the robot with the Unitree handheld remote.

Estimated time: ~20 minutes the first time (most of it docker build).
After that, ~1 minute to start.

> **Safety first.** AMO is a real locomotion policy on a real humanoid.
> Keep one hand on the **E-stop** of the Unitree remote at all times,
> work in an open area, and never run it on a robot you cannot catch
> with a safety harness or gantry.

---

## Prerequisites

- A G1 on a stand or with a safety harness, powered on, in **damp** mode.
- The Unitree handheld remote, paired.
- A Linux PC with Docker, plugged into the G1's onboard switch via the
  NIC the scripts expect (`enxc8a362edcebb` by default).
- A booted X11 desktop session (so we can see the dashboard).

If anything above is non-default for you, read
[DOCKER.md](DOCKER.md) before continuing.

---
## Step 0 — modify the interfaces to connect your computer to the g1 with a lan cable
Go and read the [INTERFACE.md](INTERFACE.md) file

## Step 1 — Build the Docker image (one-time)

In a terminal on your Linux PC:

```bash
cd /path/to/policypilot
./docker/build.sh
```

Grab a coffee — this takes 5–20 min the first time. Subsequent rebuilds
are much faster thanks to layer caching.

## Step 2 — Start the container

```bash
./docker/run.sh
```

You'll see the script set up the robot NIC, then drop you into a `bash`
shell inside the container at `/ros2_ws`.

Inside the container, your environment is already ready:

```text
# inside container
root@host:/ros2_ws# echo $ROS_DOMAIN_ID
1
root@host:/ros2_ws# echo $RMW_IMPLEMENTATION
rmw_cyclonedds_cpp
```

## Step 3 — Build policypilot (one-time per source change)

```bash
# inside container
cd /ros2_ws
colcon build --packages-select policypilot
source install/setup.bash
```

You can re-`source` the same file in any new terminal you open inside
the container.

## Step 4 — Bring everything up

```bash
# inside container
ros2 launch policypilot bringup_launcher.launch.py
```

This starts:

- `robot_state`         — publishes joint states + TF + IMU
- `loco_client`         — talks to the Unitree loco RPC server
- `arm_controller`, `dx3_controller`, `interactive_marker`
- `policy_manager`      — the RoboJuDo supervisor (idle, waiting)
- `control_panel`       — the PyQt6 **dashboard window** (this is the one
  you'll click)
- RViz2                 — the visualizer
- Livox driver          — only if `nodes.livox_launcher_enabled` is true

The dashboard appears on your host display. The status bar at the bottom
reads `POLICY: STOPPED | idle` — that means `policy_manager` has not
spawned RoboJuDo yet.

If the window does not appear, see
[DOCKER.md §5](DOCKER.md#5-gui-visualizing-rviz-the-dashboard-etc).

## Step 5 — Start AMO walking

On the dashboard:

```
  ┌──────────┬───────────┬───────────┬──────────┬──────────┐
  │  START   │ START     │ START +   │   AMO    │  STOP    │
  │          │ BALANCING │ BALANCE   │   WALK   │  POLICY  │
  └──────────┴───────────┴───────────┴──────────┴──────────┘
```



1. Press **`AMO WALK`**.
   Start it with the robot hanging on the harness and the feet slightly touching the ground.
   The robot should balance and after you can control it with the unitree controller.
   This process takes up to a minute. Don't stand close to the robot while performing this operation.

   Under the hood, `policy_manager` ran:

   ```bash
   /opt/policypilot-runtime/bin/python  \
       /opt/policypilot/policy_runtime/scripts/run_pipeline.py  \
       -c g1_amo_real --iface enxc8a362edcebb
   ```

   (see [ROBOJUDO_INTEGRATION.md](ROBOJUDO_INTEGRATION.md) for the full
   chain.)

3. **Drive with the Unitree handheld remote.**
   The AMO policy reads the wireless remote *directly from the DDS bus*,
   no ROS bridge needed:

   | Stick | Effect |
   | --- | --- |
   | Left stick — forward / back | linear vx |
   | Left stick — left / right | linear vy |
   | Right stick — left / right | yaw rate |
   | Right stick — up / down | (depends on the policy build; commonly height) |
   | L2 / R2 + face buttons | Unitree-side mode buttons (see Unitree docs) |

   Move the left stick *gently*. The robot will start stepping.

## Step 6 — Stop cleanly

- Press **`STOP POLICY`** on the dashboard.
  This sends SIGTERM to the AMO subprocess; the status bar reads
  `POLICY: STOPPED | stopped (stop requested)`. The robot keeps the
  Unitree BalanceStand it inherited.
- Or press **`EMERGENCY STOP`** (bottom-right, red).
  This stops the policy *and* asks loco_client to damp. Use this if
  anything feels wrong.

To shut down everything, `Ctrl+C` the `bringup_launcher` terminal, then
`exit` the container.

---

## What just happened (one paragraph)

`bringup_launcher` started a normal ROS 2 graph (state + loco + arms +
dashboard + a quiet `policy_manager`). When you clicked **AMO WALK**, the
dashboard published `True` on `/policypilot/policy/start`. `policy_manager`
read that, spawned `run_pipeline.py -c g1_amo_real` under the
`policypilot-runtime` conda env, and let it talk DDS to the robot at 120 Hz. The
AMO policy reads the Unitree wireless remote off the bus, computes joint
targets, and writes them back — that's the walking. When you clicked
**STOP POLICY**, the dashboard published `True` on
`/policypilot/policy/stop` and `policy_manager` sent SIGTERM to the whole
subprocess group.

---

## Where to go next

| If you want to… | Read |
| --- | --- |
| Understand the architecture | [ARCHITECTURE.md](ARCHITECTURE.md) |
| See the exact CLI / env vars used by `policy_manager` | [ROBOJUDO_INTEGRATION.md](ROBOJUDO_INTEGRATION.md) |
| Swap to a different RoboJuDo pipeline (e.g. `g1_amo_arm_teleop_real`) | [CONFIGURATION.md](CONFIGURATION.md) — `policy.config_name` |
| Run from a CLI instead of the dashboard | [ROS_NODES.md](ROS_NODES.md) — `policy_manager` topics |
| Skip Docker and run on a bare Linux PC | [DOCKER.md §"What is in the image"](DOCKER.md#what-is-in-the-image) |
