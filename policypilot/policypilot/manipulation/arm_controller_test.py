#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import socket
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple


def _default_config_path() -> Path:
    env_cfg = os.getenv("POLICYPILOT_CONFIG")
    if env_cfg:
        p = Path(env_cfg).expanduser().resolve()
        if p.exists():
            return p

    try:
        from ament_index_python.packages import get_package_share_directory
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

    # Keep a deterministic fallback for error messages.
    return here.parents[2] / "config" / "config.yaml"


def _load_config(path: Path) -> Dict[str, Any]:
    try:
        import yaml
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "PyYAML is required to read config files. Install `python3-yaml` in your ROS environment."
        ) from exc

    with path.open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected a YAML object in {path}, got: {type(data).__name__}")
    return data


def _available_interfaces() -> set[str]:
    return {name for _, name in socket.if_nameindex()}


def _ethernet_candidates(interfaces: Iterable[str]) -> list[str]:
    prefixes = ("eth", "en", "eno", "ens", "enp", "enx")
    return sorted(
        iface for iface in interfaces if iface != "lo" and iface.startswith(prefixes)
    )


def _route_interface_to_ip(robot_ip: str) -> Optional[str]:
    try:
        result = subprocess.run(
            ["ip", "route", "get", robot_ip],
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError:
        return None

    if result.returncode != 0:
        return None

    parts = result.stdout.strip().split()
    if "dev" not in parts:
        return None
    idx = parts.index("dev")
    if idx + 1 >= len(parts):
        return None
    return parts[idx + 1]


def _ping_reachable(robot_ip: str, interface: str, timeout_s: float) -> Optional[bool]:
    # Linux ping syntax; returns None when ping is unavailable in PATH.
    timeout_i = max(1, int(timeout_s))
    try:
        result = subprocess.run(
            ["ping", "-I", interface, "-c", "1", "-W", str(timeout_i), robot_ip],
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError:
        return None
    return result.returncode == 0


def _resolve_interface(
    config_interface: str,
    robot_ip: str,
    probe_timeout_s: float,
    probe_connectivity: bool,
) -> Tuple[str, str]:
    interfaces = _available_interfaces()
    route_iface = _route_interface_to_ip(robot_ip)
    candidates = _ethernet_candidates(interfaces)

    if not interfaces:
        raise RuntimeError("No network interfaces detected on this machine.")

    if config_interface in interfaces:
        if not probe_connectivity:
            return config_interface, "Using interface from config (probe disabled)."

        reachable = _ping_reachable(robot_ip, config_interface, probe_timeout_s)
        if reachable is True:
            return config_interface, f"Config interface reaches robot IP {robot_ip}."
        if reachable is None:
            return config_interface, "Ping unavailable; using interface from config."

    if route_iface and route_iface in interfaces:
        if not probe_connectivity:
            return route_iface, f"Using routed interface to {robot_ip}."

        reachable = _ping_reachable(robot_ip, route_iface, probe_timeout_s)
        if reachable is True:
            return route_iface, f"Route-selected interface reaches robot IP {robot_ip}."
        if reachable is None:
            return route_iface, "Ping unavailable; using routed interface."

    for iface in candidates:
        if iface == config_interface or iface == route_iface:
            continue
        if not probe_connectivity:
            return iface, "Using first Ethernet-like interface as fallback."

        reachable = _ping_reachable(robot_ip, iface, probe_timeout_s)
        if reachable is True:
            return iface, f"Discovered reachable Ethernet interface for {robot_ip}."
        if reachable is None:
            return iface, "Ping unavailable; using Ethernet fallback interface."

    if config_interface in interfaces:
        return config_interface, "Falling back to config interface (reachability failed)."
    if route_iface and route_iface in interfaces:
        return route_iface, "Falling back to route-selected interface."
    if candidates:
        return candidates[0], "Falling back to first Ethernet-like interface."

    raise RuntimeError(
        f"Could not resolve a valid interface. Config requested '{config_interface}', "
        f"available={sorted(interfaces)}"
    )


def _bool_to_ros(value: bool) -> str:
    return "true" if value else "false"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run arm_controller only, using policypilot config and G1 interface auto-selection."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=_default_config_path(),
        help="Path to config.yaml (default: repository config/config.yaml).",
    )
    parser.add_argument(
        "--interface",
        type=str,
        default=None,
        help="Override interface from config (for example: eth0, enp3s0).",
    )
    parser.add_argument(
        "--robot-ip",
        type=str,
        default="192.168.123.161",
        help="Expected robot IPv4 for route/ping checks.",
    )
    parser.add_argument(
        "--probe-timeout",
        type=float,
        default=1.0,
        help="Per-interface ping timeout in seconds.",
    )
    parser.add_argument(
        "--no-probe",
        action="store_true",
        help="Disable connectivity probing and trust config/route only.",
    )
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Force simulation mode (use_robot:=false).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show resolved parameters and exit without starting the node.",
    )
    args = parser.parse_args()

    if not args.config.exists():
        raise FileNotFoundError(f"Config file not found: {args.config}")

    cfg = _load_config(args.config)
    general = cfg.get("general", {})
    if not isinstance(general, dict):
        general = {}

    use_robot = bool(general.get("use_robot", True))
    if args.sim:
        use_robot = False

    config_iface = str(args.interface or general.get("interface", "eth0"))

    if use_robot:
        interface, reason = _resolve_interface(
            config_interface=config_iface,
            robot_ip=args.robot_ip,
            probe_timeout_s=args.probe_timeout,
            probe_connectivity=not args.no_probe,
        )
    else:
        interface = config_iface
        reason = "Simulation mode: interface kept for parameter consistency."

    print(f"[arm_controller_test] config: {args.config}")
    print(f"[arm_controller_test] use_robot: {use_robot}")
    print(f"[arm_controller_test] interface: {interface}")
    print(f"[arm_controller_test] note: {reason}")

    ros_args = [
        "--ros-args",
        "-p",
        f"interface:={interface}",
        "-p",
        f"use_robot:={_bool_to_ros(use_robot)}",
    ]
    print(f"[arm_controller_test] ros args: {' '.join(ros_args)}")

    if args.dry_run:
        return

    import rclpy
    from policypilot.manipulation.arm_controller import ArmController

    print("[arm_controller_test] calling rclpy.init()", flush=True)
    rclpy.init(args=ros_args)
    print("[arm_controller_test] creating ArmController node...", flush=True)
    node = ArmController()
    print("[arm_controller_test] node created, spinning...", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[arm_controller_test] ERROR: {exc}", file=sys.stderr)
        raise

