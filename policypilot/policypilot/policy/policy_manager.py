#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
policy_manager
==============

ROS 2 node that supervises a single RoboJuDo pipeline subprocess.

The pipeline itself lives in ``policy_runtime/`` (the vendored RoboJuDo tree)
and runs under a separate Python interpreter (typically the
``policypilot-runtime`` conda env), so this node is purely a launcher / monitor:

* Subscribes to ``/policypilot/policy/start`` and ``/policypilot/policy/stop``
  (``std_msgs/Bool``) to spawn / terminate the pipeline.
* Subscribes to ``/policypilot/emergency_stop`` for immediate teardown.
* Publishes liveness on ``/policypilot/policy/running`` and a free-form
  ``/policypilot/policy/status`` string for the UI.

Every knob is exposed as a ROS parameter and defaulted from the
``policy:`` section of ``config/config.yaml``. The default ``config_name``
is ``g1_amo_real`` (AMO locomotion balance, no arm teleop).
"""

import os
import shlex
import signal
import subprocess
import threading
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool, String


def _default_config_path() -> Path:
    env_cfg = os.getenv("POLICYPILOT_CONFIG")
    if env_cfg:
        p = Path(env_cfg).expanduser().resolve()
        if p.exists():
            return p

    try:
        p = Path(get_package_share_directory("policypilot")) / "config" / "config.yaml"
        if p.exists():
            return p
    except Exception:
        pass

    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "config" / "config.yaml"
        if candidate.exists():
            return candidate

    return here.parents[2] / "config" / "config.yaml"


def _repo_root_candidates() -> list[Path]:
    """Candidate roots for finding the vendored policy_runtime tree."""
    candidates: list[Path] = []
    env_root = os.getenv("POLICYPILOT_ROOT")
    if env_root:
        candidates.append(Path(env_root).expanduser())

    here = Path(__file__).resolve()
    for parent in here.parents[:6]:
        candidates.append(parent)

    return candidates


def _autodetect_policy_runtime() -> Optional[Path]:
    for root in _repo_root_candidates():
        candidate = root / "policy_runtime"
        if (candidate / "scripts" / "run_pipeline.py").is_file():
            return candidate
    return None


def _load_config_defaults() -> dict:
    """
    Defaults are sized so that a fresh checkout 'just works' when
    ``policy_runtime`` sits next to the ``policypilot`` ROS package and
    the ``policypilot-runtime`` conda env is installed in
    ``/opt/policypilot-runtime``.
    """
    autodetected_runtime = _autodetect_policy_runtime()
    runtime = autodetected_runtime or Path("/opt/policypilot/policy_runtime")

    defaults = {
        # Interpreter for the RoboJuDo pipeline (NOT the ROS Python).
        "python_executable": "/opt/policypilot-runtime/bin/python",
        "conda_prefix": "/opt/policypilot-runtime",
        "mplconfigdir": "/tmp/matplotlib",

        # Vendored policy runtime (RoboJuDo tree).
        "policy_runtime": str(runtime),
        "runner_script": str(runtime / "scripts" / "run_pipeline.py"),
        "working_directory": str(runtime.parent),

        # Which RoboJuDo cfg_registry entry to run.
        "config_name": "g1_amo_real",

        # Robot networking.
        "interface": "eth0",
        "img_server_ip": "192.168.123.164",

        # Episode/run logging knobs (passed through unchanged to run_pipeline.py).
        "record": False,
        "task_dir": "/data/policypilot_runs",
        "task_name": "g1_amo_run",
        "task_goal": "",
        "task_desc": "",
        "task_steps": "",
    }

    config_path = _default_config_path()
    try:
        import yaml

        with config_path.open("r", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}

        general = data.get("general", {}) if isinstance(data, dict) else {}
        policy_section = (
            data.get("policy") or data.get("low_level") or {}
            if isinstance(data, dict) else {}
        )
        if isinstance(general, dict) and general.get("interface"):
            defaults["interface"] = str(general["interface"])
        if isinstance(policy_section, dict):
            for key in defaults:
                if key in policy_section and policy_section[key] is not None:
                    defaults[key] = policy_section[key]
    except Exception:
        pass

    return defaults


class PolicyManager(Node):
    """Spawns and supervises a RoboJuDo pipeline subprocess."""

    def __init__(self):
        super().__init__("policy_manager")

        defaults = _load_config_defaults()
        for key, value in defaults.items():
            self.declare_parameter(key, value)

        self.pub_running = self.create_publisher(Bool, "/policypilot/policy/running", 10)
        self.pub_status = self.create_publisher(String, "/policypilot/policy/status", 10)

        self.create_subscription(Bool, "/policypilot/policy/start", self._start_callback, 10)
        self.create_subscription(Bool, "/policypilot/policy/stop", self._stop_callback, 10)
        self.create_subscription(Bool, "/policypilot/emergency_stop", self._emergency_stop_callback, 10)

        self._proc_lock = threading.Lock()
        self._proc: Optional[subprocess.Popen[str]] = None
        self._stop_requested = False
        self._stop_reason = ""

        self._publish_state(False, "idle")

    # ------------------------------------------------------------------ helpers

    def _publish_state(self, running: bool, status: str):
        self.pub_running.publish(Bool(data=running))
        self.pub_status.publish(String(data=status))
        self.get_logger().info(f"[policy] running={running} status={status}")

    def _parameter_text(self, name: str) -> str:
        value = self.get_parameter(name).value
        return "" if value is None else str(value)

    def _build_command(self):
        python_executable = Path(self._parameter_text("python_executable")).expanduser()
        runner_script = Path(self._parameter_text("runner_script")).expanduser()
        policy_runtime = Path(self._parameter_text("policy_runtime")).expanduser()
        working_directory = Path(self._parameter_text("working_directory")).expanduser()
        config_name = self._parameter_text("config_name")
        interface = self._parameter_text("interface")
        conda_prefix = self._parameter_text("conda_prefix")
        mplconfigdir = self._parameter_text("mplconfigdir")

        if not python_executable.is_file():
            raise FileNotFoundError(f"python_executable not found: {python_executable}")
        if not runner_script.is_file():
            raise FileNotFoundError(f"runner_script not found: {runner_script}")
        if not policy_runtime.is_dir():
            raise FileNotFoundError(f"policy_runtime not found: {policy_runtime}")
        if not working_directory.is_dir():
            raise FileNotFoundError(f"working_directory not found: {working_directory}")

        command = [str(python_executable), str(runner_script), "-c", config_name]
        if interface:
            command.extend(["--iface", interface])
        if bool(self.get_parameter("record").value):
            command.append("--record")

        def _append_text_arg(flag: str, name: str):
            value = self._parameter_text(name)
            if value:
                command.extend([flag, value])

        _append_text_arg("--img-server-ip", "img_server_ip")
        _append_text_arg("--task-dir", "task_dir")
        _append_text_arg("--task-name", "task_name")
        _append_text_arg("--task-goal", "task_goal")
        _append_text_arg("--task-desc", "task_desc")
        _append_text_arg("--task-steps", "task_steps")

        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        if mplconfigdir:
            env["MPLCONFIGDIR"] = mplconfigdir
        if conda_prefix:
            env["CONDA_PREFIX"] = conda_prefix
            conda_lib = str(Path(conda_prefix).expanduser() / "lib")
            current_ld = env.get("LD_LIBRARY_PATH", "")
            env["LD_LIBRARY_PATH"] = (
                conda_lib if not current_ld else f"{conda_lib}{os.pathsep}{current_ld}"
            )
            conda_bin = str(Path(conda_prefix).expanduser() / "bin")
            current_path = env.get("PATH", "")
            env["PATH"] = conda_bin if not current_path else f"{conda_bin}{os.pathsep}{current_path}"

        current_pythonpath = env.get("PYTHONPATH", "")
        pythonpath_entries = [str(policy_runtime)]
        if current_pythonpath:
            pythonpath_entries.append(current_pythonpath)
        env["PYTHONPATH"] = os.pathsep.join(pythonpath_entries)
        env["ROBOJUDO_ROOT"] = str(policy_runtime)
        env["ROBOJUDO_TASK_DIR"] = self._parameter_text("task_dir")
        env.setdefault("CYCLONEDDS_HOME", "/usr/local")

        return command, env, working_directory

    # ------------------------------------------------------------- subscriptions

    def _start_callback(self, msg: Bool):
        if msg.data:
            self.start_policy()

    def _stop_callback(self, msg: Bool):
        if msg.data:
            self.stop_policy("stop requested")

    def _emergency_stop_callback(self, msg: Bool):
        if msg.data:
            self.stop_policy("emergency stop")

    # ---------------------------------------------------------------- lifecycle

    def start_policy(self):
        with self._proc_lock:
            if self._proc is not None and self._proc.poll() is None:
                self._publish_state(True, "already running")
                return
            self._proc = None
            self._stop_requested = False
            self._stop_reason = ""

        try:
            command, env, working_directory = self._build_command()
            self.get_logger().info(f"[policy] starting command: {shlex.join(command)}")
            proc = subprocess.Popen(
                command,
                cwd=str(working_directory),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except Exception as exc:
            self._publish_state(False, f"start failed: {exc}")
            return

        with self._proc_lock:
            self._proc = proc

        self._publish_state(True, f"started pid={proc.pid}")
        threading.Thread(target=self._stream_output, args=(proc,), daemon=True).start()
        threading.Thread(target=self._watch_process, args=(proc,), daemon=True).start()

    def _stream_output(self, proc: subprocess.Popen[str]):
        if proc.stdout is None:
            return
        try:
            for raw_line in proc.stdout:
                line = raw_line.rstrip()
                if line:
                    self.get_logger().info(f"[policy] {line}")
        except Exception as exc:
            self.get_logger().warning(f"[policy] output reader stopped: {exc}")

    def _watch_process(self, proc: subprocess.Popen[str]):
        returncode = proc.wait()
        with self._proc_lock:
            if self._proc is proc:
                self._proc = None
            stop_requested = self._stop_requested
            stop_reason = self._stop_reason
            self._stop_requested = False
            self._stop_reason = ""

        if stop_requested:
            status = f"stopped ({stop_reason})"
        elif returncode == 0:
            status = "exited cleanly"
        else:
            status = f"exited with code {returncode}"
        self._publish_state(False, status)

    def stop_policy(self, reason: str):
        with self._proc_lock:
            proc = self._proc
            if proc is None or proc.poll() is not None:
                self._publish_state(False, f"not running ({reason})")
                return
            self._stop_requested = True
            self._stop_reason = reason

        self._publish_state(True, f"stopping ({reason})")
        threading.Thread(target=self._terminate_process, args=(proc,), daemon=True).start()

    def _terminate_process(self, proc: subprocess.Popen[str]):
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except Exception:
            try:
                proc.terminate()
            except Exception:
                return

        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warning("[policy] process did not stop after SIGTERM, sending SIGKILL")
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass

    def shutdown(self):
        with self._proc_lock:
            proc = self._proc
            if proc is None or proc.poll() is not None:
                return
            self._stop_requested = True
            self._stop_reason = "node shutdown"

        self._publish_state(True, "stopping (node shutdown)")
        self._terminate_process(proc)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
