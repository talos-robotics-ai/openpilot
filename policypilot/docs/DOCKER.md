# Docker Setup

This is the recommended way to run policypilot: a single self-contained
image with ROS 2 Humble, the `policypilot-runtime` conda env, the Unitree SDK,
Livox/MOLA, PyQt6, RViz2 and the policy_runtime tree all baked in.

If you would rather install everything by hand on your Linux PC, the
[package list](#what-is-in-the-image) at the bottom tells you what to
install (apt + pip + conda + sources).

---

## 1. Prerequisites (host PC, Linux)

- **Docker Engine** with current user in the `docker` group
  *(or willing to run `sudo`)*.
- A **Linux desktop with X11** running. (Wayland works via XWayland, but
  you may need `xhost +SI:localuser:root` once per session.)
- A dedicated **NIC connected to the G1's onboard switch**. Default name
  in the scripts is `enxc8a362edcebb` with host IP `192.168.123.222/24` —
  override with `POLICYPILOT_ROBOT_INTERFACE=...` and
  `POLICYPILOT_ROBOT_HOST_CIDR=...` if your wiring differs.
- About **10 GB free disk** for the image (CUDA-class deps, conda env,
  vendored RoboJuDo assets).
- (Optional) **NVIDIA Container Toolkit** if you want GPU access; the
  default image is CPU-friendly but PyTorch will run faster with CUDA.

---

## 2. Build the image

```bash
cd policypilot
./docker/build.sh
```

This runs:

```bash
docker build -t policypilot:latest \
    -f docker/Dockerfile  \
    <policypilot repo root>
```

The build context is the `policypilot/` root (not the parent
`controlBoard/`), so the `Dockerfile` reaches `COPY policy_runtime/` and
`COPY docker/vendor/` inline. Expect 5–20 min depending on bandwidth — the
conda env, Livox SDK build, ROS packages, MOLA and Torch are the long
parts.

Override the image tag with:

```bash
POLICYPILOT_IMAGE_TAG=policypilot:dev ./docker/build.sh
```

---

## 3. Start the container

Once built:

```bash
./docker/run.sh
```

This script:

1. Configures the robot NIC (`ip addr replace ${ROBOT_HOST_CIDR} dev
   ${ROBOT_INTERFACE}`) — set `POLICYPILOT_SKIP_NET_CONFIG=1` to skip.
2. Mounts your `policypilot/` checkout into `/ros2_ws/src/policypilot`,
   so file edits on the host take effect immediately (you only need to
   re-run `colcon build` inside the container).
3. Mounts `policypilot/runs/` (created if missing) onto
   `${ROBOJUDO_TASK_DIR}` (`/data/policypilot_runs`) so episode logs
   persist on the host.
4. Forwards `$DISPLAY` + the X cookie for GUI apps.
5. Starts the container with `--net host --privileged` so DDS works on
   the Unitree bus.

Useful overrides (env vars consumed by `run.sh`):

| Var | Default | Purpose |
| --- | --- | --- |
| `POLICYPILOT_IMAGE_TAG` | `policypilot:latest` | Image to run |
| `POLICYPILOT_CONTAINER_NAME` | `policypilot` | Container name |
| `POLICYPILOT_ROBOT_INTERFACE` | `enxc8a362edcebb` | Host NIC bound to the robot |
| `POLICYPILOT_ROBOT_HOST_CIDR` | `192.168.123.222/24` | Host IP on that NIC |
| `POLICYPILOT_SKIP_NET_CONFIG` | `0` | Skip the host IP/up bring-up |
| `POLICYPILOT_RUNS_DIR` | `$REPO/runs` | Where episode logs land on the host |

You land in `bash`. The login profile has already sourced
`/opt/ros/humble/setup.bash` and `/ros2_ws/install/setup.bash`, and set
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` plus `ROS_DOMAIN_ID=1`.

To reattach later:

```bash
docker exec -it policypilot bash
```

To stop and remove:

```bash
docker stop policypilot && docker rm policypilot
```

---

## 4. First build of policypilot inside the container

The host checkout is mounted in at `/ros2_ws/src/policypilot`. Build it
once:

```bash
cd /ros2_ws
colcon build --packages-select policypilot
source install/setup.bash
```

Re-run `colcon build` whenever you change `setup.py` / launch files / new
modules; for pure Python source edits the install symlinks usually mean
you only need to restart the node.

---

## 5. GUI: visualizing RViz, the dashboard, etc.

The container shares the host's X server. Two GUI surfaces matter:

- **RViz2** (`ros2 launch policypilot robot_state_launcher.launch.py` or
  via `bringup_launcher`) — the URDF + LiDAR scene viewer.
- **control_panel** (`ros2 run policypilot control_panel`, or auto-started
  by `bringup_launcher`) — the PyQt6 button grid with `AMO WALK`,
  `START BALANCING`, hands, `EMERGENCY STOP`, etc.

If a window does *not* appear (you see `Could not connect to display :0`):

```bash
# On the host (one-time per session):
xhost +SI:localuser:root
```

For SSH-from-another-machine forwarding, run the container with
`-e DISPLAY=$DISPLAY` and use `ssh -Y`; or expose your X server over
TCP and set `DISPLAY=host_ip:0`. The `run.sh` script already wires up
`XAUTHORITY` from your host cookie.

For headless boxes there is no built-in fallback — install a virtual
framebuffer (`xvfb-run`) or run the dashboard from another machine that
shares the same `ROS_DOMAIN_ID` and DDS interface.

---

## 6. Verify the image is healthy

Inside the container:

```bash
# ROS 2 alive?
ros2 doctor --report --include-warnings 2>&1 | head

# Cyclone DDS bindings load?
python3 -c "import cyclonedds; print(cyclonedds.__file__)"

# policypilot-runtime env intact?
${POLICYPILOT_RT_PYTHON} -c "import torch, mujoco, robojudo; print('OK')"

# Can policy_manager discover the vendored runtime?
ros2 run policypilot policy_manager --ros-args -p config_name:=g1_amo_real &
sleep 2
ros2 topic echo /policypilot/policy/status --once
```

If `python3 -c "import cyclonedds"` fails with a `libddsc.so` error,
re-run `ldconfig` and verify `/usr/local/lib/libddsc.so.0` is the vendor
symlink (the Dockerfile sets this up; if your image is older, rebuild).

---

## What is in the image

If you want to reproduce the environment on a bare Linux PC (no Docker),
here is the inventory.

### apt packages (Ubuntu Jammy / ROS Humble base)

ROS 2:
`ros-humble-xacro`, `ros-humble-rviz2`, `ros-humble-pcl-conversions`,
`ros-humble-joy`, `ros-humble-rmw-cyclonedds-cpp`,
`ros-humble-rosidl-default-generators`,
`ros-humble-mola`, `ros-humble-mola-state-estimation`,
`ros-humble-mola-lidar-odometry`.

System:
`python3-pip`, `python3-colcon-common-extensions`, `libpcl-dev`,
`terminator`, `git`, `gedit`, `iproute2`, `iputils-ping`, `wget`,
`ca-certificates`, `bzip2`, `cyclonedds-dev`.

GUI / GL:
`libegl1`, `libgl1`, `libosmesa6`, `libsm6`, `libxext6`, `libxrender1`,
`libxkbcommon-x11-0`, `libxcb-cursor0`, `libxcb-icccm4`, `libxcb-image0`,
`libxcb-keysyms1`, `libxcb-render-util0`, `libxcb-xinerama0`,
`libxcb-xinput0`, `libxcb-shape0`.

Media / camera:
`python3-gi`, `gstreamer1.0-tools`, `gstreamer1.0-plugins-*`,
`gstreamer1.0-libav`.

### pip (host Python — used by ROS nodes)

`pyqt6`, `pin`, `meshcat`, `evdev`, `pyrealsense2`, `opencv-python`.

### conda env (`/opt/policypilot-runtime`, defined in
[`docker/policypilot-runtime.environment.yml`](../docker/policypilot-runtime.environment.yml)
and [`docker/policypilot-runtime.requirements.txt`](../docker/policypilot-runtime.requirements.txt))

Conda (python 3.11): `numpy=1.26.4`, `scipy=1.17.1`, `pyzmq=27.1.0`,
`casadi=3.7.0`, `eigenpy=3.12.0`, `coal=3.0.2`, `hpp-fcl=3.0.2`,
`pinocchio=3.9.0`.

Pip on top of the conda env: `torch==2.10.0`, `onnxruntime==1.24.3`,
`joblib==1.5.3`, `PyYAML==6.0.3`, `easydict==1.13`, `python-box==7.4.1`,
`tqdm==4.67.3`, `pygame==2.6.1`, `pynput==1.8.1`, `pydantic==2.12.5`,
`mujoco==3.5.0`, `msgpack==1.1.2`, `msgpack-numpy==0.4.8`,
`colorlog==6.10.1`, `opencv-python==4.11.0.86`, `pyarrow==23.0.1`,
`matplotlib==3.10.8`, `meshcat==0.3.2`, `evdev==1.9.3`.

Plus editable installs of `unitree_sdk2_python` (pinned commit
`7c661d27`) and the bundled `policy_runtime/` itself.

### Native libraries built from source

- **Livox-SDK2** (`git clone Livox-SDK/Livox-SDK2`, `cmake && make install`).
- **livox_ros_driver2** (built via `colcon` inside `/ros2_ws`).
- **astroviz_interfaces** (`git clone CDonosoK/astroviz_interfaces`,
  built via `colcon`).

### Vendor blobs (`docker/vendor/`)

- `libddsc.so` — patched CycloneDDS C library with Unitree's IDLs
  baked in. Installed into `/usr/local/lib` and symlinked as
  `libddsc.so.0`.
- `cyclonedds/` + `cyclonedds-0.10.2.dist-info/` — matching Python
  bindings, copied into the policypilot-runtime site-packages.

### Host-side mounts

- `/ros2_ws/src/policypilot` ← host `policypilot/` checkout
- `${ROBOJUDO_TASK_DIR}` (default `/data/policypilot_runs`) ← host
  `policypilot/runs/`
- `/ros2_ws/src/livox_ros_driver2/config/MID360_config.json` ← host
  `policypilot/config/livox_mid.json` *(only if that file exists)*

---

## Troubleshooting

| Symptom | Likely cause / fix |
| --- | --- |
| `docker build` fails at `COPY policy_runtime` | You ran `docker build` from the wrong directory. Use `./docker/build.sh` or pass `policypilot/` as context |
| `Could not connect to display` on RViz/PyQt | Run `xhost +SI:localuser:root` once on the host; or check `$DISPLAY` is set |
| `rmw_cyclonedds_cpp` errors at startup | The vendor `libddsc.so` is missing. Rebuild the image, or copy it manually to `/usr/local/lib/` and run `ldconfig` |
| `policy_manager` reports `start failed: python_executable not found` | The `policypilot-runtime` env was not installed in the image. Rebuild from a clean state |
| No DDS traffic visible (`ros2 topic list` empty) | The container is on the wrong NIC. Check `POLICYPILOT_ROBOT_INTERFACE` and that the host NIC is up |
| Container can't get an IP on the robot NIC | The `ip addr replace` step needs sudo. Either run the script as root or accept the `sudo` prompts |
