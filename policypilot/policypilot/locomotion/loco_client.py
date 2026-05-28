
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import threading
import time
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Joy

from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_api import (
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
)


def _rpc_get_int(client, api_id):
    try:
        code, data = client._Call(api_id, "{}")
        if code == 0 and data:
            return json.loads(data).get("data")
    except Exception:
        pass
    return None


class G1LocoClient(Node):
    def __init__(self):
        super().__init__("loco_client")
        self.robot_stopped = False
        self.balanced = False
        self.prev_buttons = {}
        self.prev_axis_last = None
        self.control_arms = False

        self.declare_parameter('use_robot', True)
        self.use_robot = bool(self.get_parameter('use_robot').value)

        self.declare_parameter('interface', 'eth0')
        interface = self.get_parameter('interface').get_parameter_value().string_value
        self.declare_parameter('dds_domain_id', 0)
        self.dds_domain_id = int(self.get_parameter('dds_domain_id').value)
        self.declare_parameter('arm_controlled', 'both')
        self.arm_controlled = self.get_parameter('arm_controlled').get_parameter_value().string_value
        self.declare_parameter('enable_arm_ui', True)
        self.enable_arm_ui = self.get_parameter('enable_arm_ui').get_parameter_value().bool_value

        self.declare_parameter('ik_use_waist', False)
        self.declare_parameter('ik_alpha', 0.2)
        self.declare_parameter('ik_max_dq_step', 0.05)
        self.declare_parameter('arm_velocity_limit', 2.0)

        ik_use_waist = self.get_parameter('ik_use_waist').get_parameter_value().bool_value
        ik_alpha = float(self.get_parameter('ik_alpha').value)
        ik_max_dq_step = float(self.get_parameter('ik_max_dq_step').value)
        arm_vel_lim = float(self.get_parameter('arm_velocity_limit').value)

        if self.use_robot:
            available_ifaces = {name for _, name in socket.if_nameindex()}
            if interface not in available_ifaces:
                raise RuntimeError(
                    f"Interface '{interface}' not found. Available interfaces: {sorted(available_ifaces)}"
                )

            self.get_logger().info(
                f"[startup] initializing loco client on iface='{interface}' domain={self.dds_domain_id}"
            )
            ChannelFactoryInitialize(self.dds_domain_id, interface)
            self.robot = LocoClient()
            self.robot.SetTimeout(10.0)
            self.robot.Init()
            self.current_id, self.current_mode = self._wait_fsm_ready(max_wait_s=6.0)

            if self.current_id is None or self.current_mode is None:
                self.get_logger().error(
                    "Loco RPC not reachable (FSM ID/mode unavailable). "
                    "Check interface/domain and robot loco service state."
                )
            else:
                self.get_logger().info(
                    f"[startup] loco RPC ready | FSM ID={self.current_id}, Mode={self.current_mode}"
                )

            try:
                self.robot.Damp()
            except Exception as e:
                self.get_logger().warn(f"Damp command failed during startup: {e}")
            self.get_logger().info(f"Current FSM ID: {self.current_id}, Mode: {self.current_mode}")
        else:
            self.robot = None
            self.current_id = 4
            self.current_mode = 0
            self.get_logger().info("use_robot:=false -> Not connecting to robot.")

        self.create_subscription(Bool, '/policypilot/emergency_stop', self.emergency_callback, 10)
        self.create_subscription(Bool, '/policypilot/start', self.start_callback, 10)
        self.create_subscription(Bool, '/policypilot/start_balancing', self.start_balancing_callback, 10)
        self.create_subscription(Joy, '/policypilot/joy', self.joystick_callback, 10)

        self.publisher_arms_controlled = self.create_publisher(Bool, '/policypilot/arms/enabled', 1)
        self.right_gripper_pub = self.create_publisher(String, '/policypilot/dx3/hand_action/right', 1)
        self.left_gripper_pub = self.create_publisher(String, '/policypilot/dx3/hand_action/left', 1)
        self.publisher_homming_arms = self.create_publisher(Bool, '/policypilot/arms/home', 1)

    def _log_once(self, level, msg, key):
        if getattr(self, key, False):
            return
        setattr(self, key, True)

        logger = self.get_logger()
        if level == "info":
            logger.info(msg)
        elif level == "warn":
            logger.warn(msg)
        elif level == "error":
            logger.error(msg)
        elif level == "debug":
            logger.debug(msg)
        else:
            logger.info(msg)


    def _clear_once(self, key):
        if hasattr(self, key):
            delattr(self, key)

    def _btn_rising(self, msg, idx):
        prev = self.prev_buttons.get(idx, 0)
        return msg.buttons[idx] == 1 and prev == 0

    def _btn_falling(self, msg, idx):
        prev = self.prev_buttons.get(idx, 0)
        return msg.buttons[idx] == 0 and prev == 1

    def _axis_edge(self, cur, prev, val):
        return cur == val and prev != val, cur != val and prev == val

    def get_fsm_id(self):
        if not self.use_robot or self.robot is None:
            return 4
        return _rpc_get_int(self.robot, ROBOT_API_ID_LOCO_GET_FSM_ID)

    def get_fsm_mode(self):
        if not self.use_robot or self.robot is None:
            return 0
        return _rpc_get_int(self.robot, ROBOT_API_ID_LOCO_GET_FSM_MODE)

    def _wait_fsm_ready(self, max_wait_s: float = 5.0):
        t0 = time.time()
        while (time.time() - t0) < max_wait_s:
            fsm_id = self.get_fsm_id()
            fsm_mode = self.get_fsm_mode()
            if fsm_id is not None and fsm_mode is not None:
                return fsm_id, fsm_mode
            self.get_logger().warn(
                f"[startup] waiting for loco RPC... {(time.time() - t0):.1f}s elapsed"
            )
            time.sleep(0.5)
        return None, None

    def _wait_for_fsm_id(self, expected_ids, max_wait_s: float = 4.0, poll_s: float = 0.2):
        if isinstance(expected_ids, int):
            expected_ids = {expected_ids}
        else:
            expected_ids = set(expected_ids)

        t0 = time.time()
        while (time.time() - t0) < max_wait_s:
            fsm_id = self.get_fsm_id()
            if fsm_id in expected_ids:
                return True
            time.sleep(poll_s)
        return False

    def _enter_locked_stand(self):
        if not self.use_robot or self.robot is None:
            self.robot_stopped = False
            self.balanced = False
            return True

        # Official G1 sport-service transition requires passing through damp/standby
        # before entering sport mode. Going directly from damp to sport can be rejected.
        self.robot.Damp()
        time.sleep(0.2)
        self.robot.SetFsmId(4)
        if not self._wait_for_fsm_id(4, max_wait_s=4.0):
            self.get_logger().error(
                "[balance] Failed to reach FSM ID 4 (locked stand). "
                "The robot may not be in a locomotion-ready state."
            )
            return False

        self.robot_stopped = False
        self.balanced = False
        self.get_logger().info("[balance] Reached FSM ID 4 (locked stand).")
        return True

    def emergency_callback(self, msg: Bool):
        if msg.data:
            self._log_once("warn", "EMERGENCY STOP ACTIVATED!", "_e_stop_activated_logged")
            self.robot_stopped = True
            self.balanced = False
            if self.use_robot and self.robot is not None:
                self.robot.Damp()
            if self.control_arms:
                self.control_arms = False
                self.publisher_arms_controlled.publish(Bool(data=False))
        else:
            self._clear_once("_e_stop_activated_logged")

    def start_callback(self, msg: Bool):
        if self.use_robot and self.robot is not None and msg.data:
            if self._enter_locked_stand():
                self._log_once("info", "Switched to FSM ID 4 (locked stand)", "_switch_fsm_id_4_logged")

    def start_balancing_callback(self, msg: Bool):
        if msg.data and not self.balanced:
            self._log_once("info", "Starting balancing procedure...", "_start_balance_req_logged")
            self.entering_balancing()
            if self.balanced:
                self._log_once("info", "Balancing procedure completed.", "_balance_completed_logged")
        elif self.balanced:
            self._log_once("info", "Already balanced, no action taken.", "_already_balanced_notice_logged")

    def joystick_callback(self, msg: Joy):
        try:
            if not self.prev_buttons:
                self.prev_buttons = {i: 0 for i in range(len(msg.buttons))}

            if not self.balanced:
                self._log_once("warn", "Robot is not balanced, cannot move.", "_warn_not_balanced_logged")
            else:
                self._clear_once("_warn_not_balanced_logged")

            if self.robot_stopped:
                self._log_once("warn", "Robot is stopped, cannot move.", "_warn_robot_stopped_logged")
            else:
                self._clear_once("_warn_robot_stopped_logged")

            axis_last = msg.axes[-1] if len(msg.axes) else 0.0
            if self.prev_axis_last is None:
                self.prev_axis_last = axis_last

            up_on, up_off = self._axis_edge(axis_last, self.prev_axis_last, -1.0)
            if up_on:
                if self._enter_locked_stand():
                    self._log_once("info", "Switched to FSM ID 4 (locked stand)", "_switch_fsm_id_4_logged")
            if up_off:
                self._clear_once("_switch_fsm_id_4_logged")

            if self._btn_rising(msg, 0):
                self.control_arms = not self.control_arms
                if self.control_arms:
                    self._log_once("info", "Enabling arm control mode.", "_enable_arm_control_logged")
                    self.publisher_arms_controlled.publish(Bool(data=True))
                else:
                    self._log_once("info", "Disabling arm control mode.", "_disable_arm_control_logged")
                    self.publisher_arms_controlled.publish(Bool(data=False))

            if self._btn_rising(msg, 1):
                if self.control_arms:
                    self._log_once("info", "Moving arms to home position.", "_move_arms_home_logged")
                    self.publisher_homming_arms.publish(Bool(data=True))
                else:
                    self._log_once("warn", "Cannot move arms to home, arm control mode is disabled.", "_warn_move_home_no_control_logged")

            if self._btn_rising(msg, 5):
                self._log_once("warn", "Emergency stop button pressed!", "_e_stop_button_pressed_logged")
                self.robot_stopped = True
                self.balanced = False
                if self.use_robot and self.robot is not None:
                    self.robot.Damp()
                self.control_arms = False
                self.publisher_arms_controlled.publish(Bool(data=False))
            if self._btn_falling(msg, 5):
                self._clear_once("_e_stop_button_pressed_logged")

            # Gripper controls
            if msg.axes[4] ==1.0 and self._btn_rising(msg, 3):
                self._log_once("info", "Open right gripper.", "_open_right_gripper_logged")
                self.right_gripper_pub.publish(String(data="open"))
            if msg.axes[4] ==1.0 and self._btn_falling(msg, 3):
                self._log_once("info", "Close right gripper.", "_close_right_gripper_logged")
                self.right_gripper_pub.publish(String(data="close"))

            if msg.axes[4] ==-1.0 and self._btn_rising(msg, 3):
                self._log_once("info", "Open left gripper.", "_open_left_gripper_logged")
                self.left_gripper_pub.publish(String(data="open"))
            if msg.axes[4] ==-1.0 and self._btn_falling(msg, 3):
                self._log_once("info", "Close left gripper.", "_close_left_gripper_logged")
                self.left_gripper_pub.publish(String(data="close"))

            if self._btn_rising(msg, 6):
                if not self.balanced:
                    self._log_once("info", "Starting balancing procedure...", "_start_balance_r1_logged")
                    self.entering_balancing()
                    if self.balanced:
                        self._log_once("info", "Balancing procedure completed.", "_balance_completed_r1_logged")
                else:
                    self._log_once("info", "Already balanced, no action taken.", "_already_balanced_notice_r1_logged")
            if self._btn_falling(msg, 6):
                self._clear_once("_start_balance_r1_logged")
                self._clear_once("_balance_completed_r1_logged")
                self._clear_once("_already_balanced_notice_r1_logged")

            if msg.buttons[8] == 0 and not self.robot_stopped and self.balanced:
                if self.use_robot and self.robot is not None:
                    self.robot.StopMove()

            if msg.buttons[8] == 1 and not self.robot_stopped and self.balanced:
                vx = round(msg.axes[1] * -0.5, 2)
                vy = round(msg.axes[0] * -0.5, 2)
                yaw = round(msg.axes[2] * -0.5, 2)
                self._log_once("info", f"Moving with vx: {vx}, vy: {vy}, yaw: {yaw}", "_moving_logged")
                if self.use_robot and self.robot is not None:
                    if abs(vx) < 0.03 and abs(vy) < 0.03 and abs(yaw) < 0.03:
                        self.robot.StopMove()
                    else:
                        self.robot.Move(vx=vx, vy=vy, vyaw=yaw, continous_move=True)

            self.prev_buttons = {i: msg.buttons[i] for i in range(len(msg.buttons))}
            self.prev_axis_last = axis_last

        except Exception as e:
            self.get_logger().error(f"Error in joystick_callback: {e}")
            if self.use_robot and self.robot is not None:
                self.robot.StopMove()
                self.robot.Damp()
            self.robot_stopped = True
            self.balanced = False

    def entering_balancing(self):
        if not self.use_robot or self.robot is None:
            self.balanced = True
            self.get_logger().info("Sim balancing done (use_robot:=false).")
            return

        if not self._wait_for_fsm_id(4, max_wait_s=1.0):
            if not self._enter_locked_stand():
                self.balanced = False
                return

        # Official G1 sport-service transition is 1 -> 4 -> 500.
        # Enter sport mode explicitly instead of using the legacy height-ramp path.
        self.robot.SetFsmId(500)
        if not self._wait_for_fsm_id({500, 801}, max_wait_s=5.0):
            self.get_logger().error(
                "[balance] Failed to enter sport mode (expected FSM ID 500/801)."
            )
            self.balanced = False
            return

        # Keep stand-balance mode active once sport mode is ready.
        self.robot.BalanceStand(0)
        self.robot_stopped = False
        self.balanced = True
        self.get_logger().info(
            f"[balance] Sport mode ready | FSM ID={self.get_fsm_id()}, Mode={self.get_fsm_mode()}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = G1LocoClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


