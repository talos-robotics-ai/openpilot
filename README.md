# openpilot — Talos Robotics AI

This repository is the umbrella for **Talos Robotics AI**'s humanoid
control stack. It bundles the application-layer software we run on
Unitree-class humanoids (currently the G1), grouped by package.

Each subdirectory is a self-contained project with its own README,
build system and documentation — `openpilot` itself is just the index.

---

## Packages

### [`policypilot/`](policypilot/)

ROS 2 application layer for the Unitree G1, with the **RoboJuDo** RL
policy framework vendored as a sibling runtime (`policy_runtime/`).

- **ROS side:** state, locomotion, manipulation, a PyQt6 operator
  dashboard, and a `policy_manager` node that supervises the RoboJuDo
  subprocess.
- **Policy side:** `policy_runtime/` ships RoboJuDo configs/controllers/
  policies for the G1, including an `g1_amo_real` AMO locomotion
  pipeline that is driven by the Unitree handheld remote.
- **Docker:** a single self-contained image (`policypilot:latest`) with
  ROS 2 Humble, the `policypilot-runtime` conda env, Livox/MOLA, the
  Unitree SDK, PyQt6 and RViz2.

**Read first:**
- [`policypilot/README.md`](policypilot/README.md) — package overview.
- [`policypilot/docs/QUICKSTART.md`](policypilot/docs/QUICKSTART.md) —
  Docker → dashboard → AMO walk in five steps.
- [`policypilot/docs/ARCHITECTURE.md`](policypilot/docs/ARCHITECTURE.md)
  — design decisions and the two-runtimes-one-bus model.
- [`policypilot/docs/DOCKER.md`](policypilot/docs/DOCKER.md) — build,
  X11 forwarding, and the full package list for replicating the
  environment on a bare Linux PC.
- [`policypilot/docs/ROBOJUDO_INTEGRATION.md`](policypilot/docs/ROBOJUDO_INTEGRATION.md)
  — how `policy_manager` spawns the RoboJuDo pipeline (env vars,
  process group, lifecycle).

---

## Repository layout

```
openpilot/                       ← this repository
├── README.md                    ← you are here
└── policypilot/                 ← G1 ROS + RoboJuDo runtime
    ├── README.md
    ├── docs/                    ← architecture, docker, quickstart, …
    ├── policypilot/             ← ROS 2 ament_python package
    ├── policy_runtime/          ← vendored RoboJuDo (RL framework)
    ├── launch/
    ├── config/
    ├── description_files/
    ├── docker/
    ├── package.xml
    └── setup.py
```

Each package's `docs/` directory holds the detailed reference for that
package; this top-level README does not duplicate it.

---

## Getting started

The recommended path for a fresh checkout is:

```bash
git clone https://github.com/talos-robotics-ai/openpilot.git
cd openpilot/policypilot
./docker/build.sh        # ~5–20 min the first time
./docker/run.sh          # drops you into the container
# inside the container:
cd /ros2_ws && colcon build --packages-select policypilot && source install/setup.bash
ros2 launch policypilot bringup_launcher.launch.py
```

Then click **AMO WALK** on the PyQt dashboard that opens, and drive the
robot with the Unitree handheld remote. The full walkthrough is in
[`policypilot/docs/QUICKSTART.md`](policypilot/docs/QUICKSTART.md).

---

## Conventions for new packages in this repo

When adding a new package as a sibling to `policypilot/`:

1. Keep it **self-contained** — its own README, build, docs, and (if
   relevant) Docker setup.
2. **Don't share top-level config** with other packages. The whole
   point of the umbrella layout is that packages can evolve
   independently.
3. Link to its README from the **Packages** section above so it's
   discoverable from this index.
4. If two packages need to talk to each other, do it over ROS topics /
   DDS / files on disk — not by importing each other's Python modules.

---


