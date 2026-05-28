#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
control_panel
=============

PyQt6 operator dashboard for policypilot. Single-window grid of buttons that
publish lifecycle ROS topics — the same surface as the original g1pilot
"streamdeck" panel, restyled for the policypilot topic layout.

Buttons (label -> action):

    START                 → /policypilot/start                       (loco_client)
    START BALANCING       → /policypilot/start_balancing             (Unitree BalanceStand)
    START + BALANCE       → START then BALANCING in a short sequence
    AMO WALK              → /policypilot/policy/start                (spawn RoboJuDo AMO policy)
    STOP POLICY           → /policypilot/policy/stop                 (kill the RoboJuDo subprocess)
    HOMING ARMS           → /policypilot/arms/home
    ENABLE MANIPULATION   → /policypilot/arms/enabled                (toggle)
    OPEN / CLOSE HAND     → /policypilot/dx3/hand_action/{left,right}
    EMERGENCY STOP        → /policypilot/emergency_stop + /policypilot/policy/stop

The AMO WALK button is the headline action for the beginner flow: pressing
it starts the AMO locomotion policy through policy_manager, which spawns
policy_runtime/scripts/run_pipeline.py -c g1_amo_real. The robot is then
driven by the Unitree handheld remote (the AMO controller reads it directly
from the DDS bus — no extra ROS joystick needed).
"""

import sys

from PyQt6.QtWidgets import (
    QApplication, QWidget, QGridLayout, QPushButton, QVBoxLayout, QLabel
)
from PyQt6.QtCore import QTimer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from policypilot.utils.window_style import DarkStyle


class ControlPanelNode(Node):
    """Owns publishers / subscriptions; no GUI code lives here."""

    def __init__(self):
        super().__init__('control_panel')

        # Locomotion (Unitree loco RPC, via loco_client)
        self.pub_start = self.create_publisher(Bool, '/policypilot/start', 10)
        self.pub_start_balancing = self.create_publisher(Bool, '/policypilot/start_balancing', 10)

        # Manipulation
        self.pub_arms_enabled = self.create_publisher(Bool, '/policypilot/arms/enabled', 10)
        self.pub_arms_home = self.create_publisher(Bool, '/policypilot/arms/home', 10)
        self.pub_left_hand = self.create_publisher(String, '/policypilot/dx3/hand_action/left', 10)
        self.pub_right_hand = self.create_publisher(String, '/policypilot/dx3/hand_action/right', 10)

        # Policy lifecycle (spawns RoboJuDo via policy_manager)
        self.pub_policy_start = self.create_publisher(Bool, '/policypilot/policy/start', 10)
        self.pub_policy_stop = self.create_publisher(Bool, '/policypilot/policy/stop', 10)

        # Safety
        self.pub_emergency_stop = self.create_publisher(Bool, '/policypilot/emergency_stop', 10)

        # Mirror the policy_manager's state so the GUI can highlight it.
        self.policy_running = False
        self.policy_status = "idle"
        self.create_subscription(Bool,   '/policypilot/policy/running', self._on_running, 10)
        self.create_subscription(String, '/policypilot/policy/status',  self._on_status,  10)

    def _topic_name(self, pub) -> str:
        return getattr(pub, "topic_name", "<unknown>")

    def publish_bool(self, pub, value: bool):
        msg = Bool(data=value)
        pub.publish(msg)
        self.get_logger().info(f"[ui] {self._topic_name(pub)} -> {value}")

    def publish_str(self, pub, text: str):
        msg = String(data=text)
        pub.publish(msg)
        self.get_logger().info(f"[ui] {self._topic_name(pub)} -> {text!r}")

    def _on_running(self, msg: Bool):
        self.policy_running = bool(msg.data)

    def _on_status(self, msg: String):
        self.policy_status = str(msg.data)


class ControlPanelGUI(QWidget):
    """5x5 button grid wired to a ControlPanelNode."""

    def __init__(self, ros_node: ControlPanelNode):
        super().__init__()
        self.node = ros_node
        self.button_states: dict[tuple[int, int], bool] = {}
        self.hand_pairs = {
            "left":  {"open": (2, 0), "close": (2, 1)},
            "right": {"open": (2, 2), "close": (2, 3)},
        }

        self.setWindowTitle("policypilot — control panel")
        self._init_ui()
        self._apply_style()
        self._last_policy_running = None

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_policy_status)
        self.status_timer.start(200)

    # ------------------------------------------------------------------ layout
    def _init_ui(self):
        main_layout = QVBoxLayout()
        grid = QGridLayout()
        grid.setSpacing(10)
        rows, cols = 5, 5
        self.buttons: dict[tuple[int, int], QPushButton] = {}
        self.status_label = QLabel("POLICY: idle")

        button_actions = {
            (0, 0): ("START",
                     lambda: self._flash((0, 0), self.node.pub_start)),
            (0, 1): ("START\nBALANCING",
                     lambda: self._flash((0, 1), self.node.pub_start_balancing)),
            (0, 2): ("START +\nBALANCE",
                     lambda: self._start_balance_sequence((0, 2))),
            (0, 3): ("AMO\nWALK",
                     lambda: self._flash((0, 3), self.node.pub_policy_start)),
            (0, 4): ("STOP\nPOLICY",
                     lambda: self._flash((0, 4), self.node.pub_policy_stop)),

            (1, 0): ("ENABLE\nMANIPULATION",
                     lambda: self._toggle((1, 0), self.node.pub_arms_enabled)),
            (1, 1): ("HOMING\nARMS",
                     lambda: self._flash((1, 1), self.node.pub_arms_home)),

            (2, 0): ("OPEN\nLEFT\nHAND",
                     lambda: self._toggle_hand("left", "open", self.node.pub_left_hand)),
            (2, 1): ("CLOSE\nLEFT\nHAND",
                     lambda: self._toggle_hand("left", "close", self.node.pub_left_hand)),
            (2, 2): ("OPEN\nRIGHT\nHAND",
                     lambda: self._toggle_hand("right", "open", self.node.pub_right_hand)),
            (2, 3): ("CLOSE\nRIGHT\nHAND",
                     lambda: self._toggle_hand("right", "close", self.node.pub_right_hand)),

            (4, 4): ("EMERGENCY\nSTOP",
                     self._emergency_stop),
        }

        for r in range(rows):
            for c in range(cols):
                btn = QPushButton()
                btn.setMinimumSize(120, 80)

                action = button_actions.get((r, c))
                if action is None:
                    btn.setEnabled(False)
                    btn.setFlat(True)
                    btn.setStyleSheet(
                        "QPushButton { background-color:#1e1e1e; border:1px solid #333; border-radius:10px; }"
                    )
                else:
                    label, func = action
                    btn.setText(label)
                    btn.clicked.connect(func)
                    if (r, c) == (4, 4):
                        btn.setStyleSheet(self._emergency_style())

                grid.addWidget(btn, r, c)
                self.buttons[(r, c)] = btn
                self.button_states[(r, c)] = False

        main_layout.addLayout(grid)
        main_layout.addWidget(self.status_label)
        self.setLayout(main_layout)

    def _emergency_style(self) -> str:
        return """
            QPushButton { background-color:#b00000; color:white; font-weight:bold;
                          border:1px solid #ff4444; border-radius:10px; }
            QPushButton:hover { background-color:#ff0000; border:1px solid #ff6666; }
        """

    def _apply_style(self):
        self.setStyleSheet("""
            QPushButton { background-color:#2d2d2d; color:#fff; font-size:15px; font-weight:600;
                          border:1px solid #444; border-radius:10px; padding:10px; }
            QPushButton:hover:enabled { background-color:#3c3c3c; border:1px solid #66b3ff; }
            QPushButton:pressed { background-color:#1f5fa1; border:1px solid #80c4ff; }
            QPushButton:disabled { color:#555; background-color:#1e1e1e; border:1px solid #2a2a2a; }
            QWidget { background-color:#111; }
            QLabel  { color:#ddd; font-size:14px; font-weight:600; padding:6px; }
        """)

    # ---------------------------------------------------------------- helpers
    def _set_active(self, pos, active=True):
        btn = self.buttons[pos]
        if active:
            btn.setStyleSheet(
                "QPushButton { background-color:#4CAF50; color:white; font-weight:bold;"
                " border:1px solid #80ff80; border-radius:10px; }"
            )
        else:
            btn.setStyleSheet("")
            self._apply_style()
        self.button_states[pos] = active

    def _flash(self, pos, pub, duration_ms=1000):
        self._set_active(pos, True)
        self.node.publish_bool(pub, True)
        timer = QTimer(self)
        timer.setSingleShot(True)
        timer.timeout.connect(lambda: self._deactivate(pos, pub))
        timer.start(duration_ms)

    def _deactivate(self, pos, pub):
        self._set_active(pos, False)
        self.node.publish_bool(pub, False)

    def _start_balance_sequence(self, pos):
        self._set_active(pos, True)
        self.node.get_logger().info("[ui] start+balance sequence")
        self._flash((0, 0), self.node.pub_start, duration_ms=700)
        timer = QTimer(self)
        timer.setSingleShot(True)
        timer.timeout.connect(
            lambda: self._flash((0, 1), self.node.pub_start_balancing, duration_ms=1000)
        )
        timer.timeout.connect(lambda: self._set_active(pos, False))
        timer.start(450)

    def _toggle(self, pos, pub):
        new_state = not self.button_states[pos]
        self._set_active(pos, new_state)
        self.node.publish_bool(pub, new_state)

    def _toggle_hand(self, side, action, pub):
        pair = self.hand_pairs[side]
        this_pos = pair[action]
        other_pos = pair["close" if action == "open" else "open"]
        self._set_active(this_pos, True)
        self._set_active(other_pos, False)
        self.node.publish_str(pub, action)

    def _emergency_stop(self):
        """Publishes False to every Bool topic + True to stop topics."""
        self.node.publish_bool(self.node.pub_start, False)
        self.node.publish_bool(self.node.pub_start_balancing, False)
        self.node.publish_bool(self.node.pub_arms_enabled, False)
        self.node.publish_bool(self.node.pub_arms_home, False)
        self.node.publish_bool(self.node.pub_policy_stop, True)
        self.node.publish_bool(self.node.pub_emergency_stop, True)

        for pos in self.buttons:
            if pos != (4, 4):
                self._set_active(pos, False)

        self.buttons[(4, 4)].setStyleSheet(self._emergency_style())

    def _refresh_policy_status(self):
        running = bool(self.node.policy_running)
        status = self.node.policy_status or "idle"
        self.status_label.setText(
            f"POLICY: {'RUNNING' if running else 'STOPPED'} | {status}"
        )
        if running != self._last_policy_running:
            self._set_active((0, 3), running)        # AMO WALK
            if not running:
                self._set_active((0, 4), False)      # STOP POLICY
            self._last_policy_running = running


def main():
    rclpy.init()
    node = ControlPanelNode()

    app = QApplication(sys.argv)
    DarkStyle(app)
    gui = ControlPanelGUI(node)
    gui.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    app.exec()
    node.destroy_node()
    rclpy.shutdown()
    app.quit()


if __name__ == '__main__':
    main()
