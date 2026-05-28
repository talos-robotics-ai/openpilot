#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import time
from dataclasses import dataclass
from typing import Optional

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_api import (
    ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
    ROBOT_API_ID_LOCO_GET_STAND_HEIGHT,
    ROBOT_API_ID_LOCO_GET_SWING_HEIGHT,
    ROBOT_API_ID_LOCO_SET_BALANCE_MODE,
    ROBOT_API_ID_LOCO_SET_FSM_ID,
    ROBOT_API_ID_LOCO_SET_STAND_HEIGHT,
    ROBOT_API_ID_LOCO_SET_SWING_HEIGHT,
    ROBOT_API_ID_LOCO_SET_VELOCITY,
)
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

try:
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
except Exception:  # pragma: no cover - optional on some environments
    MotionSwitcherClient = None


@dataclass
class Snapshot:
    motion_mode: Optional[dict]
    fsm_id: Optional[int]
    fsm_mode: Optional[int]
    balance_mode: Optional[int]
    stand_height: Optional[float]
    swing_height: Optional[float]


class SportServiceTester:
    def __init__(self, iface: str, domain: int, rpc_timeout_s: float):
        ChannelFactoryInitialize(domain, iface)
        self.iface = iface
        self.domain = domain

        self.client = LocoClient()
        self.client.SetTimeout(rpc_timeout_s)
        self.client.Init()

        self.motion_switcher = None
        if MotionSwitcherClient is not None:
            try:
                self.motion_switcher = MotionSwitcherClient()
                self.motion_switcher.SetTimeout(1.0)
                self.motion_switcher.Init()
            except Exception:
                self.motion_switcher = None

    def _rpc_get_value(self, api_id: int):
        try:
            code, data = self.client._Call(api_id, "{}")  # type: ignore[attr-defined]
            if code == 0 and data:
                return json.loads(data).get("data")
        except Exception:
            pass
        return None

    def _rpc_set_value(self, api_id: int, value):
        payload = json.dumps({"data": value})
        return self.client._Call(api_id, payload)  # type: ignore[attr-defined]

    def _rpc_set_velocity(self, vx: float, vy: float, omega: float, duration: float):
        payload = json.dumps(
            {"velocity": [vx, vy, omega], "duration": duration}
        )
        return self.client._Call(ROBOT_API_ID_LOCO_SET_VELOCITY, payload)  # type: ignore[attr-defined]

    def snapshot(self) -> Snapshot:
        motion_mode = None
        if self.motion_switcher is not None:
            try:
                status, result = self.motion_switcher.CheckMode()
                motion_mode = {"status": status, "result": result}
            except Exception:
                motion_mode = None

        return Snapshot(
            motion_mode=motion_mode,
            fsm_id=self._rpc_get_value(ROBOT_API_ID_LOCO_GET_FSM_ID),
            fsm_mode=self._rpc_get_value(ROBOT_API_ID_LOCO_GET_FSM_MODE),
            balance_mode=self._rpc_get_value(ROBOT_API_ID_LOCO_GET_BALANCE_MODE),
            stand_height=self._rpc_get_value(ROBOT_API_ID_LOCO_GET_STAND_HEIGHT),
            swing_height=self._rpc_get_value(ROBOT_API_ID_LOCO_GET_SWING_HEIGHT),
        )

    def print_snapshot(self, label: str) -> Snapshot:
        snap = self.snapshot()
        print(
            f"[sport_service_test] {label} | "
            f"motion_mode={snap.motion_mode} "
            f"fsm_id={snap.fsm_id} "
            f"fsm_mode={snap.fsm_mode} "
            f"balance_mode={snap.balance_mode} "
            f"stand_height={snap.stand_height} "
            f"swing_height={snap.swing_height}"
        )
        return snap

    def poll_status(self, label: str, wait_s: float, poll_s: float):
        if wait_s <= 0:
            return
        deadline = time.time() + wait_s
        while time.time() < deadline:
            self.print_snapshot(label)
            time.sleep(poll_s)

    def call(self, label: str, fn, *args, **kwargs):
        try:
            result = fn(*args, **kwargs)
        except Exception as exc:
            print(f"[sport_service_test] {label} -> EXCEPTION {exc}")
            return None, None

        if isinstance(result, tuple) and len(result) == 2:
            code, data = result
        else:
            code, data = result, None

        print(f"[sport_service_test] {label} -> code={code} data={data}")
        return code, data

    def sdk_damp(self):
        return self.call("sdk Damp()", self.client.Damp)

    def sdk_start(self):
        return self.call("sdk Start() [FSM 200]", self.client.Start)

    def sdk_balance_mode(self, mode: int):
        return self.call(f"sdk BalanceStand({mode})", self.client.BalanceStand, mode)

    def sdk_set_fsm(self, fsm_id: int):
        return self.call(f"sdk SetFsmId({fsm_id})", self.client.SetFsmId, fsm_id)

    def sdk_set_stand_height(self, stand_height: float):
        return self.call(
            f"sdk SetStandHeight({stand_height})",
            self._rpc_set_value,
            ROBOT_API_ID_LOCO_SET_STAND_HEIGHT,
            stand_height,
        )

    def sdk_set_swing_height(self, swing_height: float):
        return self.call(
            f"sdk SetSwingHeight({swing_height})",
            self._rpc_set_value,
            ROBOT_API_ID_LOCO_SET_SWING_HEIGHT,
            swing_height,
        )

    def sdk_move(self, vx: float, vy: float, omega: float, duration: float):
        return self.call(
            f"sdk SetVelocity(vx={vx}, vy={vy}, omega={omega}, duration={duration})",
            self._rpc_set_velocity,
            vx,
            vy,
            omega,
            duration,
        )

    def sdk_stop(self):
        return self.call("sdk StopMove()", self.client.StopMove)

    def sdk_start_balance(self, balance_mode: int):
        # This follows the public sport-service primitives: start + balance mode.
        self.sdk_start()
        return self.sdk_balance_mode(balance_mode)

    def policypilot_start(self, settle_s: float):
        # This matches the current policypilot dashboard/start path:
        # Damp -> FSM 4.
        self.sdk_damp()
        time.sleep(settle_s)
        return self.sdk_set_fsm(4)

    def policypilot_balance(self, settle_s: float, balance_mode: int):
        # This matches the current policypilot balancing path:
        # FSM 500 -> BalanceStand(mode).
        self.sdk_set_fsm(500)
        time.sleep(settle_s)
        return self.sdk_balance_mode(balance_mode)

    def policypilot_start_balance(self, settle_s: float, balance_mode: int):
        self.policypilot_start(settle_s)
        time.sleep(settle_s)
        return self.policypilot_balance(settle_s, balance_mode)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Direct tester for the Unitree G1 'sport' RPC service. "
            "Use this to bypass the ROS dashboard and compare the public SDK API "
            "against the custom policypilot start/balance sequence."
        )
    )
    parser.add_argument(
        "--iface",
        required=True,
        help="DDS network interface for the robot, for example: enxc8a362edcebb",
    )
    parser.add_argument(
        "--domain",
        type=int,
        default=0,
        help="DDS domain id. Default: 0",
    )
    parser.add_argument(
        "--rpc-timeout",
        type=float,
        default=2.0,
        help="Per-RPC timeout in seconds. Default: 2.0",
    )
    parser.add_argument(
        "--after-wait",
        type=float,
        default=3.0,
        help="Seconds to keep polling state after sending the command. Default: 3.0",
    )
    parser.add_argument(
        "--poll",
        type=float,
        default=0.5,
        help="Polling interval in seconds after sending the command. Default: 0.5",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=0.5,
        help="Delay between steps in multi-step sequences. Default: 0.5",
    )

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("status", help="Read and print the current sport-service state.")
    sub.add_parser("sdk-damp", help="Call the official SDK Damp() helper.")
    sub.add_parser("sdk-start", help="Call the official SDK Start() helper (FSM 200).")

    p_sdk_balance = sub.add_parser(
        "sdk-balance",
        help="Call the official SDK BalanceStand(mode) helper.",
    )
    p_sdk_balance.add_argument("--mode", type=int, default=0, help="Balance mode. Default: 0")

    p_sdk_start_balance = sub.add_parser(
        "sdk-start-balance",
        help="Call official SDK Start() then BalanceStand(mode).",
    )
    p_sdk_start_balance.add_argument("--mode", type=int, default=0, help="Balance mode. Default: 0")

    p_set_fsm = sub.add_parser("sdk-set-fsm", help="Call SetFsmId(fsm_id) directly.")
    p_set_fsm.add_argument("--fsm-id", type=int, required=True)

    p_set_stand = sub.add_parser(
        "sdk-set-stand-height",
        help="Call SetStandHeight(height) directly.",
    )
    p_set_stand.add_argument("--height", type=float, required=True)

    p_set_swing = sub.add_parser(
        "sdk-set-swing-height",
        help="Call SetSwingHeight(height) directly.",
    )
    p_set_swing.add_argument("--height", type=float, required=True)

    p_move = sub.add_parser("sdk-move", help="Call SetVelocity(vx, vy, omega, duration).")
    p_move.add_argument("--vx", type=float, default=0.0)
    p_move.add_argument("--vy", type=float, default=0.0)
    p_move.add_argument("--omega", type=float, default=0.0)
    p_move.add_argument("--duration", type=float, default=1.0)

    sub.add_parser("sdk-stop", help="Call StopMove().")

    sub.add_parser(
        "policypilot-start",
        help="Run the same start sequence used by policypilot: Damp() then SetFsmId(4).",
    )

    p_policypilot_balance = sub.add_parser(
        "policypilot-balance",
        help="Run the same balance sequence used by policypilot: SetFsmId(500) then BalanceStand(mode).",
    )
    p_policypilot_balance.add_argument("--mode", type=int, default=0, help="Balance mode. Default: 0")

    p_policypilot_start_balance = sub.add_parser(
        "policypilot-start-balance",
        help="Run policypilot start then policypilot balance.",
    )
    p_policypilot_start_balance.add_argument("--mode", type=int, default=0, help="Balance mode. Default: 0")

    return parser


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()

    tester = SportServiceTester(
        iface=args.iface,
        domain=args.domain,
        rpc_timeout_s=args.rpc_timeout,
    )

    print(
        f"[sport_service_test] Initialized on iface={args.iface} domain={args.domain} "
        f"rpc_timeout={args.rpc_timeout}s"
    )
    tester.print_snapshot("before")

    if args.command == "status":
        return 0
    if args.command == "sdk-damp":
        tester.sdk_damp()
    elif args.command == "sdk-start":
        tester.sdk_start()
    elif args.command == "sdk-balance":
        tester.sdk_balance_mode(args.mode)
    elif args.command == "sdk-start-balance":
        tester.sdk_start_balance(args.mode)
    elif args.command == "sdk-set-fsm":
        tester.sdk_set_fsm(args.fsm_id)
    elif args.command == "sdk-set-stand-height":
        tester.sdk_set_stand_height(args.height)
    elif args.command == "sdk-set-swing-height":
        tester.sdk_set_swing_height(args.height)
    elif args.command == "sdk-move":
        tester.sdk_move(args.vx, args.vy, args.omega, args.duration)
    elif args.command == "sdk-stop":
        tester.sdk_stop()
    elif args.command == "policypilot-start":
        tester.policypilot_start(args.settle)
    elif args.command == "policypilot-balance":
        tester.policypilot_balance(args.settle, args.mode)
    elif args.command == "policypilot-start-balance":
        tester.policypilot_start_balance(args.settle, args.mode)
    else:  # pragma: no cover - argparse keeps this unreachable
        raise ValueError(f"Unsupported command: {args.command}")

    tester.poll_status("after", args.after_wait, args.poll)
    tester.print_snapshot("final")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
