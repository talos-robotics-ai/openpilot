#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import time
from typing import Optional

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_api import (
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
)
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


def _rpc_get_int(client: LocoClient, api_id: int) -> Optional[int]:
    try:
        code, data = client._Call(api_id, "{}")  # type: ignore[attr-defined]
        if code == 0 and data:
            return json.loads(data).get("data")
    except Exception:
        pass
    return None


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only probe for the Unitree loco RPC. "
            "This checks whether FSM ID/mode can be queried on the selected DDS interface."
        )
    )
    parser.add_argument(
        "--iface",
        required=True,
        help="Network interface for Unitree DDS (for example: enxc8a362edcebb).",
    )
    parser.add_argument(
        "--domain",
        type=int,
        default=0,
        help="DDS domain ID for the robot. Default: 0.",
    )
    parser.add_argument(
        "--wait",
        type=float,
        default=6.0,
        help="Total seconds to wait for FSM ID/mode responses.",
    )
    parser.add_argument(
        "--rpc-timeout",
        type=float,
        default=10.0,
        help="LocoClient RPC timeout in seconds.",
    )
    parser.add_argument(
        "--poll",
        type=float,
        default=0.5,
        help="Polling period in seconds while waiting for replies.",
    )
    args = parser.parse_args()

    print(
        f"[loco_rpc_test] ChannelFactoryInitialize(domain={args.domain}, iface={args.iface})"
    )
    ChannelFactoryInitialize(args.domain, args.iface)

    client = LocoClient()
    client.SetTimeout(args.rpc_timeout)
    client.Init()
    print("[loco_rpc_test] LocoClient initialized")

    deadline = time.time() + args.wait
    while time.time() < deadline:
        fsm_id = _rpc_get_int(client, ROBOT_API_ID_LOCO_GET_FSM_ID)
        fsm_mode = _rpc_get_int(client, ROBOT_API_ID_LOCO_GET_FSM_MODE)
        if fsm_id is not None and fsm_mode is not None:
            print(f"[loco_rpc_test] PASS fsm_id={fsm_id} fsm_mode={fsm_mode}")
            return 0

        elapsed = args.wait - max(deadline - time.time(), 0.0)
        print(
            f"[loco_rpc_test] waiting for loco RPC... {elapsed:.1f}s elapsed "
            f"(fsm_id={fsm_id}, fsm_mode={fsm_mode})"
        )
        time.sleep(args.poll)

    print("[loco_rpc_test] FAIL no loco RPC response")
    print("[loco_rpc_test] HINT if rt/lowstate works but this fails:")
    print("[loco_rpc_test] HINT robot is likely still in debug mode, not in normal locomotion mode, or the loco service is not active.")
    print("[loco_rpc_test] HINT exit debug mode or reboot the robot control stack before retrying.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
