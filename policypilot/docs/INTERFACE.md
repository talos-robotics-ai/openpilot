# Changing the Robot Network Interface

This document lists every place that hard-codes the host NIC bound to the
Unitree DDS bus, and what to rebuild after editing each one. The reference
value used in the rest of the docs is `enp12s0`; substitute your own.

## TL;DR

For a normal interface swap on an already-built Docker image:

1. Edit [config/config.yaml](../config/config.yaml) — change both `general.interface` and `policy.interface`.
2. (Optional) Edit [docker/run.sh](../docker/run.sh) `ROBOT_INTERFACE` default, or pass `POLICYPILOT_ROBOT_INTERFACE=<iface>` when launching.
3. Rebuild the ROS package inside the container:
   ```bash
   docker exec policypilot bash -c \
     "source /opt/ros/humble/setup.bash && \
      cd /ros2_ws && \
      colcon build --packages-select policypilot --symlink-install"
   ```
   With `--symlink-install`, subsequent edits to `config/config.yaml` take
   effect on the next launch without another rebuild.
4. Restart the launch (`ros2 launch policypilot ...`).

That covers the runtime path. The image-baked defaults in `policy_runtime/`
are overridden at runtime via `--iface`, so you only need to touch them if
you also rebuild the Docker image (see "Docker image rebuild" below).

---

## Authoritative settings (must edit)

These are the values actually consumed at launch time.

| File | Line | Key | Notes |
|---|---|---|---|
| [config/config.yaml](../config/config.yaml) | 5 | `general.interface` | Used by `loco_client`, `robot_state`, `dx3_hand`, and other ROS nodes that initialise `ChannelFactoryInitialize`. |
| [config/config.yaml](../config/config.yaml) | 43 | `policy.interface` | Forwarded by `policy_manager` to `run_pipeline.py` as `--iface`. Overrides the baked default in `policy_runtime/robojudo/config/g1/g1_cfg.py`. |

After editing: rebuild policypilot (see TL;DR step 3) unless the install
share path is already a symlink to source (check with
`ls -la /ros2_ws/install/policypilot/share/policypilot/config/config.yaml`).

---

## Host-side network bring-up

| File | Line | Setting | Notes |
|---|---|---|---|
| [docker/run.sh](../docker/run.sh) | 25 | `ROBOT_INTERFACE` default | Used by `run.sh` to assign `192.168.123.222/24` to the NIC before starting the container. Skip with `POLICYPILOT_SKIP_NET_CONFIG=1`. |
| [docker/run.sh](../docker/run.sh) | 11 | Comment | Update for accuracy if you change the default. |

Override without editing:
```bash
POLICYPILOT_ROBOT_INTERFACE=<iface> ./docker/run.sh
```

---

## Source defaults (overridden at runtime, but should match)

These hard-coded defaults are baked into the Docker image at build time.
They are normally overridden by `--iface` from `policy_manager`, but it is
worth keeping them in sync so that running `run_pipeline.py` directly (no
ROS layer) still works.

| File | Line | Symbol | Default |
|---|---|---|---|
| [policy_runtime/robojudo/config/g1/g1_cfg.py](../policy_runtime/robojudo/config/g1/g1_cfg.py) | 72 | `g1_real.env.unitree.net_if` | `"eth0"` (placeholder) |
| [policy_runtime/robojudo/config/g1/g1_cfg.py](../policy_runtime/robojudo/config/g1/g1_cfg.py) | 97 | `g1_amo_real.env.unitree.net_if` | active default for the AMO pipeline |
| [policy_runtime/robojudo/config/g1/g1_cfg.py](../policy_runtime/robojudo/config/g1/g1_cfg.py) | 121 | `g1_amo_arm_teleop_real.env.unitree.net_if` | active default for the AMO + arm-teleop pipeline |
| [policy_runtime/robojudo/config/g1/env/g1_real_env_cfg.py](../policy_runtime/robojudo/config/g1/env/g1_real_env_cfg.py) | 23, 36 | `G1RealEnvCfg.unitree.net_if` | base-class fallbacks |
| [policy_runtime/robojudo/environment/env_cfgs.py](../policy_runtime/robojudo/environment/env_cfgs.py) | 65 | `UnitreeCfg.net_if` | dataclass default for any env cfg that does not override |

After editing: rebuild the Docker image (see below). Without a rebuild,
the on-host edits are not seen by `/opt/policypilot/policy_runtime/`
inside the container.

---

## Test / utility scripts (only if you run them standalone)

The help-text examples reference the old NIC name. Update them only if you
want the `--help` output to match your hardware; they do not affect launch.

| File | Line |
|---|---|
| [policypilot/locomotion/loco_rpc_test.py](../policypilot/locomotion/loco_rpc_test.py) | 37 |
| [policypilot/locomotion/sport_service_test.py](../policypilot/locomotion/sport_service_test.py) | 210 |

---

## Docker image rebuild

You only need this if you changed any file under `policy_runtime/` (the
RoboJuDo tree COPYed into `/opt/policypilot/policy_runtime/` at image
build time), or anything else baked into the image.

```bash
docker stop policypilot && docker rm policypilot
./docker/build.sh
./docker/run.sh
```

The mounted source tree at `/ros2_ws/src/policypilot/` (see
[docker/run.sh:75](../docker/run.sh)) does NOT include the baked
`policy_runtime/` — the autodetect in
[policypilot/policy/policy_manager.py:75](../policypilot/policy/policy_manager.py)
honours the `POLICYPILOT_ROOT=/opt/policypilot` env var that the Dockerfile
sets, so the image's copy wins over the mounted one. Either rebuild the
image, or mount the host `policy_runtime/` over the image's path:

```bash
# Add to docker/run.sh `docker run` invocation:
-v "${POLICYPILOT_DIR}/policy_runtime:/opt/policypilot/policy_runtime"
```

---

## Sanity check after a swap

Inside the container:

```bash
# 1. Confirm the interface exists and is up.
ip -br link show enp12s0

# 2. Confirm both config keys resolve correctly.
grep -n 'interface' /ros2_ws/install/policypilot/share/policypilot/config/config.yaml

# 3. Confirm policy_manager forwards --iface.
ros2 param get /policy_manager interface
```

If the policy subprocess fails with
`python: <iface>: does not match an available interface` followed by
`ChannelFactoryInitialize` → "channel factory init error", the
`policy.interface` value did not propagate. Re-check step 2 and rerun the
colcon build.
