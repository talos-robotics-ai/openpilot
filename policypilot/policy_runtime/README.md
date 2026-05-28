# policy_runtime — Vendored RoboJuDo

This directory is a **vendored copy of the RoboJuDo RL policy framework**
that the parent ROS package (`policypilot/`) spawns as a subprocess via
the `policy_manager` node.

> See [`../docs/ROBOJUDO_INTEGRATION.md`](../docs/ROBOJUDO_INTEGRATION.md)
> for how it is invoked from ROS, and
> [`ROBOJUDO_README.md`](ROBOJUDO_README.md) for the upstream documentation.

## What's inside

| Path | Origin |
| --- | --- |
| `robojudo/` | The RoboJuDo Python package (configs, controllers, envs, policies) |
| `scripts/run_pipeline.py` | The CLI entry point that `policy_manager` spawns |
| `scripts/view_*.py` | Offline visualizers (msgpack logs, BeyondMimic motions) |
| `assets/` | G1/H1 meshes, motions, and model checkpoints (~130 MB) |
| `packages/` | Source for `unitree_cpp`, `zed_proxy` |
| `third_party/` | Patched copies of mujoco_viewer / phc helpers |
| `tests/` | Upstream RoboJuDo tests |
| `pyproject.toml`, `requirements.txt`, `submodule_*` | Install metadata |
| `ROBOJUDO_README.md` | Upstream README (renamed to avoid clashing with this file) |
| `.gitmodules` | Pointer file (submodules are not auto-checked-out) |

## Local modifications vs upstream

The vendored tree is intentionally close to upstream. The only deliberate
downstream change is:

- A new `g1_amo_real` config in
  [`robojudo/config/g1/g1_cfg.py`](robojudo/config/g1/g1_cfg.py) — AMO
  locomotion balance *without* the `ArmTeleopCtrlCfg` override. This is the
  default pipeline that `policy_manager` spawns and is what policypilot
  uses as the "balance controller."

Everything else (controllers, policies, envs, training scripts) is
upstream RoboJuDo and is updated by replacing the whole tree.

## Running it directly (no ROS)

For debugging, the runner is a regular Python CLI:

```bash
/opt/policypilot-runtime/bin/python policy_runtime/scripts/run_pipeline.py \
    -c g1_amo_real --iface enxc8a362edcebb
```

This is exactly the command line that `policy_manager` constructs, minus
the env-var setup (`PYTHONPATH`, `CONDA_PREFIX`, `LD_LIBRARY_PATH`,
`ROBOJUDO_ROOT`, etc.) — for which see
[`docs/ROBOJUDO_INTEGRATION.md`](../docs/ROBOJUDO_INTEGRATION.md#33-the-environment-overrides).

## Upgrading

1. Drop a fresh upstream RoboJuDo tree on top of this directory (preserve
   `README.md` and the `g1_amo_real` config patch in `g1_cfg.py`).
2. Verify by running the CLI above by hand.
3. Re-check `policy.python_executable` in `../config/config.yaml` against
   the new `requirements.txt`.
