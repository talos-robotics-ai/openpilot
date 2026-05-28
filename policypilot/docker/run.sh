#!/usr/bin/env bash
# Start the policypilot container.
#
# Mounts:
#   - The host policypilot/ checkout into /ros2_ws/src/policypilot
#     (so you can iterate without rebuilding the image)
#   - The host MID360 JSON into the livox driver's expected location
#   - A host run-data directory into ${ROBOJUDO_TASK_DIR}
#
# Networking: --net host + --privileged are needed because the Unitree DDS
# bus runs on a dedicated NIC (default enxc8a362edcebb @ 192.168.123.222/24).
# Set POLICYPILOT_SKIP_NET_CONFIG=1 to skip the host-side IP setup.
#
# GUI: $DISPLAY and your X cookie are forwarded so RViz2 / the dashboard
# can show on the host's screen. On the host run `xhost +local:root` once
# (or use `xhost +SI:localuser:root`) if the window doesn't appear.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
POLICYPILOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
CONTAINER_NAME="${POLICYPILOT_CONTAINER_NAME:-policypilot}"
IMAGE_TAG="${POLICYPILOT_IMAGE_TAG:-policypilot:latest}"
ROBOT_INTERFACE="${POLICYPILOT_ROBOT_INTERFACE:-enp12s0}"
ROBOT_HOST_CIDR="${POLICYPILOT_ROBOT_HOST_CIDR:-192.168.123.222/24}"
HOST_RUNS_DIR="${POLICYPILOT_RUNS_DIR:-${POLICYPILOT_DIR}/runs}"
CONTAINER_RUNS_DIR="${ROBOJUDO_TASK_DIR:-/data/policypilot_runs}"

mkdir -p "${HOST_RUNS_DIR}"

run_privileged() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

if docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
  echo "Container '${CONTAINER_NAME}' already exists." >&2
  echo "Stop/remove it first, or set POLICYPILOT_CONTAINER_NAME=... for a different name:" >&2
  echo "  docker stop ${CONTAINER_NAME} && docker rm ${CONTAINER_NAME}" >&2
  exit 1
fi

if [[ "${POLICYPILOT_SKIP_NET_CONFIG:-0}" != "1" ]]; then
  if ip link show "${ROBOT_INTERFACE}" >/dev/null 2>&1; then
    echo "[net] configuring ${ROBOT_INTERFACE} with ${ROBOT_HOST_CIDR}"
    run_privileged ip addr replace "${ROBOT_HOST_CIDR}" dev "${ROBOT_INTERFACE}"
    run_privileged ip link set "${ROBOT_INTERFACE}" up
    ip -br addr show "${ROBOT_INTERFACE}"
  else
    echo "[net] WARNING: robot interface '${ROBOT_INTERFACE}' was not found on the host." >&2
    echo "       Set POLICYPILOT_ROBOT_INTERFACE=... or POLICYPILOT_SKIP_NET_CONFIG=1 if this is intentional." >&2
  fi
fi

EXTRA_MOUNTS=()
if [[ -f "${POLICYPILOT_DIR}/config/livox_mid.json" ]]; then
  EXTRA_MOUNTS+=(-v "${POLICYPILOT_DIR}/config/livox_mid.json:/ros2_ws/src/livox_ros_driver2/config/MID360_config.json")
fi

docker run \
  -it \
  --name "${CONTAINER_NAME}" \
  --env DISPLAY="${DISPLAY}" \
  --env QT_X11_NO_MITSHM=0 \
  --env XAUTHORITY=/tmp/.Xauthority \
  -v "${XAUTH}:/tmp/.Xauthority:ro" \
  --net host \
  --privileged \
  --device-cgroup-rule='c 81:* rmw' \
  -v /dev:/dev \
  -v "${POLICYPILOT_DIR}:/ros2_ws/src/policypilot" \
  -v "${HOST_RUNS_DIR}:${CONTAINER_RUNS_DIR}" \
  "${EXTRA_MOUNTS[@]}" \
  -w /ros2_ws \
  --group-add video \
  "${IMAGE_TAG}"
