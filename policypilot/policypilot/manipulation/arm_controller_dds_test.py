#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import threading
import time
from pathlib import Path

import numpy as np
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

from policypilot.manipulation.arm_controller_test import (
    _default_config_path,
    _load_config,
    _resolve_interface,
)
from policypilot.utils.common import (
    DataBuffer,
    G1_29_JointArmIndex,
    G1_29_JointIndex,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Direct DDS test for arm_controller path (rt/arm_sdk): "
            "sub rt/lowstate and pub LowCmd arm targets."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=_default_config_path(),
        help="Path to policypilot config.yaml.",
    )
    parser.add_argument(
        "--interface",
        type=str,
        default=None,
        help="Override network interface (e.g., enp3s0).",
    )
    parser.add_argument(
        "--robot-ip",
        type=str,
        default="192.168.123.164",
        help="Robot IPv4 used for route/ping interface auto-selection.",
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
        "--duration",
        type=float,
        default=8.0,
        help="Total test duration (seconds).",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=250.0,
        help="DDS publish rate in Hz.",
    )
    parser.add_argument(
        "--move-joint",
        type=int,
        default=7,
        help="Arm joint slot to excite [0..13] (default=right shoulder pitch).",
    )
    parser.add_argument(
        "--amplitude",
        type=float,
        default=0.08,
        help="Sine amplitude in radians. Set 0 for hold-only (no movement).",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=0.25,
        help="Sine frequency in Hz for the excited joint.",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=2.0,
        help="Initial hold-only time before sine excitation (seconds).",
    )
    parser.add_argument(
        "--motion-threshold",
        type=float,
        default=0.02,
        help="Measured joint delta threshold (rad) used for PASS/FAIL.",
    )
    parser.add_argument(
        "--wait-lowstate-timeout",
        type=float,
        default=8.0,
        help="Timeout waiting for first rt/lowstate packet.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Resolve settings and print plan without touching DDS.",
    )
    return parser.parse_args()


def _resolve_iface(args: argparse.Namespace) -> str:
    if not args.config.exists():
        raise FileNotFoundError(f"Config file not found: {args.config}")

    cfg = _load_config(args.config)
    general = cfg.get("general", {})
    if not isinstance(general, dict):
        general = {}

    config_iface = str(args.interface or general.get("interface", "eth0"))
    interface, reason = _resolve_interface(
        config_interface=config_iface,
        robot_ip=args.robot_ip,
        probe_timeout_s=args.probe_timeout,
        probe_connectivity=not args.no_probe,
    )
    print(f"[arm_controller_dds_test] config={args.config}")
    print(f"[arm_controller_dds_test] interface={interface}")
    print(f"[arm_controller_dds_test] note={reason}")
    return interface


def _joint_arm_vector_from_lowstate(msg: LowState_) -> np.ndarray:
    return np.array([msg.motor_state[jid.value].q for jid in G1_29_JointArmIndex], dtype=float)


def _joint_arm_vector_from_lowstate_dq(msg: LowState_) -> np.ndarray:
    return np.array([msg.motor_state[jid.value].dq for jid in G1_29_JointArmIndex], dtype=float)


def main() -> None:
    args = _parse_args()
    if not 0 <= args.move_joint <= 13:
        raise ValueError(f"--move-joint must be in [0..13], got {args.move_joint}")

    if args.rate <= 0.0:
        raise ValueError("--rate must be > 0")
    if args.duration <= 0.0:
        raise ValueError("--duration must be > 0")

    interface = _resolve_iface(args)
    print(
        "[arm_controller_dds_test] plan: "
        f"domain=0 topic_pub=rt/arm_sdk topic_sub=rt/lowstate rate={args.rate}Hz "
        f"duration={args.duration}s warmup={args.warmup}s amplitude={args.amplitude}rad "
        f"joint_slot={args.move_joint}"
    )

    if args.dry_run:
        return

    print(f"[arm_controller_dds_test] ChannelFactoryInitialize(domain=0, iface={interface})")
    ChannelFactoryInitialize(0, interface)
    print("[arm_controller_dds_test] DDS factory initialized")

    lowstate_buffer = DataBuffer()
    lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    lowstate_subscriber.Init()
    lowcmd_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
    lowcmd_publisher.Init()

    running = True

    def _sub_loop() -> None:
        while running:
            msg = lowstate_subscriber.Read()
            if msg is not None:
                lowstate_buffer.SetData(msg)
            time.sleep(0.001)

    thread = threading.Thread(target=_sub_loop, daemon=True)
    thread.start()

    t0 = time.time()
    while lowstate_buffer.GetData() is None:
        if time.time() - t0 > args.wait_lowstate_timeout:
            raise TimeoutError(
                f"No rt/lowstate received within {args.wait_lowstate_timeout}s on iface={interface}"
            )
        time.sleep(0.01)

    first = lowstate_buffer.GetData()
    assert first is not None
    base_q = _joint_arm_vector_from_lowstate(first).copy()
    base_dq = _joint_arm_vector_from_lowstate_dq(first).copy()
    print(
        "[arm_controller_dds_test] first lowstate: "
        f"mode_machine={getattr(first, 'mode_machine', 'n/a')} "
        f"arm_q={np.round(base_q, 3)} arm_dq={np.round(base_dq, 3)}"
    )

    crc = CRC()
    msg = unitree_hg_msg_dds__LowCmd_()
    # Match the official Unitree arm_sdk example message shape.
    kp_arm, kd_arm = 60.0, 1.5

    period = 1.0 / args.rate
    test_start = time.time()
    rx_count = 0
    observed_delta = 0.0
    target_joint_baseline = float(base_q[args.move_joint])
    phase_start = test_start + args.warmup
    print("[arm_controller_dds_test] running DDS command stream...")

    try:
        while True:
            now = time.time()
            elapsed = now - test_start
            if elapsed >= args.duration:
                break

            low = lowstate_buffer.GetData()
            if low is not None:
                rx_count += 1
                measured_q = _joint_arm_vector_from_lowstate(low)
                observed_delta = max(
                    observed_delta,
                    abs(float(measured_q[args.move_joint]) - target_joint_baseline),
                )

            q_cmd = base_q.copy()
            if args.amplitude != 0.0 and now >= phase_start:
                t_cmd = now - phase_start
                q_cmd[args.move_joint] = (
                    target_joint_baseline
                    + args.amplitude * math.sin(2.0 * math.pi * args.frequency * t_cmd)
                )

            try:
                msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0
            except Exception:
                pass

            for idx, jid in enumerate(G1_29_JointArmIndex):
                msg.motor_cmd[jid].q = float(q_cmd[idx])
                msg.motor_cmd[jid].dq = 0.0
                msg.motor_cmd[jid].tau = 0.0
                msg.motor_cmd[jid].kp = kp_arm
                msg.motor_cmd[jid].kd = kd_arm

            msg.crc = crc.Crc(msg)
            lowcmd_publisher.Write(msg)
            time.sleep(period)
    finally:
        running = False

    verdict = "PASS"
    if args.amplitude != 0.0 and observed_delta < args.motion_threshold:
        verdict = "FAIL"

    print("[arm_controller_dds_test] done")
    print(f"[arm_controller_dds_test] lowstate_samples={rx_count}")
    print(
        "[arm_controller_dds_test] joint_check: "
        f"slot={args.move_joint} observed_delta={observed_delta:.4f}rad "
        f"threshold={args.motion_threshold:.4f}rad"
    )
    print(f"[arm_controller_dds_test] RESULT={verdict}")

    if verdict != "PASS":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
