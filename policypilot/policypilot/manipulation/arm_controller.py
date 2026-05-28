
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, ColorRGBA
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import pinocchio as pin
from pinocchio import SE3



from policypilot.utils.joints_names import (
    JOINT_NAMES_ROS,
    JOINT_LIMITS_RAD,
    RIGHT_JOINT_INDICES_LIST,
    LEFT_JOINT_INDICES_LIST,
    WAIST_JOINT_INDICES_LIST,
)

from policypilot.utils.ik_solver import G1IKSolver

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ , LowState_ 
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from policypilot.utils.common import (
    MotorState,
    G1_29_JointArmIndex,
    G1_29_JointIndex,
    DataBuffer,
)

WORKSPACE = {
    "frame": 'pelvis',
    "left_arm": {
        "left_bottom_front": [0.33, 0.24, 0.02],
        "right_bottom_front": [0.33, 0.07,  0.02],
        "left_bottom_back":   [0.16, 0.24,  0.02],
        "right_bottom_back":  [0.16, 0.07,  0.02],
        "right_top_back":    [0.07, 0.20,  0.20],
        "left_top_back":     [0.07, 0.47,  0.20],
        "right_top_front":  [0.45, 0.11,  0.20],
        "left_top_front":   [0.41, 0.30,  0.20],
    },

    "right_arm": {
        "left_bottom_front": [0.33, -0.24, 0.02],
        "right_bottom_front": [0.33, -0.07,  0.02],
        "left_bottom_back":   [0.16, -0.24,  0.02],
        "right_bottom_back":  [0.16, -0.07,  0.02],
        "right_top_back":    [0.07, -0.20,  0.20],
        "left_top_back":     [0.07, -0.47,  0.20],
        "right_top_front":  [0.45, -0.11,  0.20],
        "left_top_front":   [0.41, -0.30,  0.20],
    },
}

DEFAULT_HOME_RIGHT = np.array([0.90, -0.06, 0.04, -0.78, -0.07, -0.11, -0.30], dtype=float)
DEFAULT_HOME_LEFT = np.array([0.90, -0.06, 0.04, -0.78, -0.07, -0.11, -0.30], dtype=float)


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


def _load_home_targets_from_config(logger):
    home_left = DEFAULT_HOME_LEFT.copy()
    home_right = DEFAULT_HOME_RIGHT.copy()
    config_path = _default_config_path()

    try:
        import yaml

        with config_path.open("r", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}

        home_cfg = data.get("home", {}) if isinstance(data, dict) else {}
        left_cfg = home_cfg.get("left", {}) if isinstance(home_cfg, dict) else {}
        right_cfg = home_cfg.get("right", {}) if isinstance(home_cfg, dict) else {}

        def _read_arm(side_cfg, fallback):
            values = []
            for idx in range(1, 8):
                key = f"motor_{idx}"
                values.append(float(side_cfg.get(key, fallback[idx - 1])))
            return np.array(values, dtype=float)

        home_left = _read_arm(left_cfg, home_left)
        home_right = _read_arm(right_cfg, home_right)
        logger.info(f"[startup] loaded arm home targets from {config_path}")
    except Exception as exc:
        logger.warning(
            f"[startup] failed to load arm home targets from {config_path}, using built-in defaults: {exc}"
        )

    return home_left, home_right


def _yaw_from_R(R: np.ndarray) -> float:
    """Yaw (Z) desde matriz de rotación."""
    return math.atan2(R[1, 0], R[0, 0])


def _mat_to_quat_wxyz(R: np.ndarray):
    q = pin.Quaternion(R)
    return np.array([q.w, q.x, q.y, q.z])


def _quat_wxyz_to_matrix(qwxyz):
    w, x, y, z = qwxyz
    return pin.Quaternion(w, x, y, z).matrix()


class ArmController(Node):
    """
    ROS 2 node controlling G1 arms with external IK (G1IKSolver) and Unitree DDS.

    This node manages the high-level control of both G1-29 arms by combining
    inverse kinematics (Pinocchio-based), filtered goal tracking, end-effector
    auto-calibration, and direct low-level command publishing via Unitree's DDS
    LowCmd interface. It also supports simulation mode through /joint_states
    publishing when `use_robot=False`.

    Parameters
    ----------
    use_robot : bool
        Enables physical robot control (DDS) if True; simulation otherwise.
    interface : str
        Ethernet interface for DDS communication.
    arm_velocity_limit : float
        Maximum allowed joint velocity.
    rate_hz : float
        Main loop update frequency.
    ik_world_frame : str
        Reference frame for IK computation.
    ik_alpha : float
        Exponential smoothing coefficient for joint updates.
    ik_goal_filter_alpha : float
        Low-pass filter coefficient for goal smoothing.
    ik_orientation_mode : str
        Orientation mode for IK ('full', 'yaw-only', etc.).
    ik_max_ori_step_rad : float
        Maximum allowed orientation step in radians per iteration.
    ee_auto_calibrate : bool
        Enables end-effector offset auto-calibration.
    auto_reissue_goals : bool
        Automatically reapply last goals after homing.
    goal_pos_tol : float
        Tolerance (m) for position convergence check.
    goal_ori_tol_deg : float
        Tolerance (deg) for orientation convergence check.
    ee_offset_right_xyz : list[float]
        Static XYZ offset for right-hand calibration.
    ee_offset_right_rpy_deg : list[float]
        Static RPY offset (deg) for right-hand calibration.
    ee_offset_left_xyz : list[float]
        Static XYZ offset for left-hand calibration.
    ee_offset_left_rpy_deg : list[float]
        Static RPY offset (deg) for left-hand calibration.
    """



    def __init__(self):
        super().__init__("arm_controller")
        self.get_logger().info("Arm Controller Node started.")

        self.declare_parameter("use_robot", True)
        self.declare_parameter("interface", "eth0")
        self.declare_parameter("arm_velocity_limit", 5.0)
        self.declare_parameter("rate_hz", 250.0)
        self.declare_parameter("ik_world_frame", "pelvis")
        self.declare_parameter("ik_use_waist", False)
        self.declare_parameter("ik_alpha", 0.2)
        self.declare_parameter("ik_max_dq_step", 0.05)
        self.declare_parameter("ik_goal_filter_alpha", 0.25)
        self.declare_parameter("ik_orientation_mode", "full")
        self.declare_parameter("ik_max_ori_step_rad", 0.35)
        self.declare_parameter("ee_auto_calibrate", True)


        self.declare_parameter("ee_offset_right_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("ee_offset_right_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("ee_offset_left_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("ee_offset_left_rpy_deg", [0.0, 0.0, 0.0])

        self.use_robot = bool(self.get_parameter("use_robot").value)
        self.interface = str(self.get_parameter("interface").value)
        self.arm_velocity_limit = float(self.get_parameter("arm_velocity_limit").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.frame = str(self.get_parameter("ik_world_frame").value)
        self.ik_use_waist = bool(self.get_parameter("ik_use_waist").value)
        self.ik_alpha = float(self.get_parameter("ik_alpha").value)
        self.ik_max_dq_step = float(self.get_parameter("ik_max_dq_step").value)
        self.ik_goal_filter_alpha = float(self.get_parameter("ik_goal_filter_alpha").value)
        self.ik_orientation_mode = str(self.get_parameter("ik_orientation_mode").value).lower()
        self.ik_max_ori_step_rad = float(self.get_parameter("ik_max_ori_step_rad").value)
        self.ee_auto_calibrate = bool(self.get_parameter("ee_auto_calibrate").value)
        self.get_logger().info(
            f"[startup] params loaded | use_robot={self.use_robot} interface={self.interface} "
            f"rate_hz={self.rate_hz} ik_use_waist={self.ik_use_waist}"
        )

        self.declare_parameter("auto_reissue_goals", True)
        self.declare_parameter("goal_pos_tol", 0.01)
        self.declare_parameter("goal_ori_tol_deg", 3.0)

        self.auto_reissue_goals = bool(self.get_parameter("auto_reissue_goals").value)
        self.goal_pos_tol = float(self.get_parameter("goal_pos_tol").value)
        self.goal_ori_tol_deg = float(self.get_parameter("goal_ori_tol_deg").value)

        def _pvec(name):
            v = self.get_parameter(name).value
            return np.array(v, dtype=float)

        self._ee_off_right_xyz = _pvec("ee_offset_right_xyz")
        self._ee_off_right_rpy_deg = _pvec("ee_offset_right_rpy_deg")
        self._ee_off_left_xyz = _pvec("ee_offset_left_xyz")
        self._ee_off_left_rpy_deg = _pvec("ee_offset_left_rpy_deg")

        self.home_left, self.home_right = _load_home_targets_from_config(self.get_logger())
        self.control_joint_indices = (
            list(WAIST_JOINT_INDICES_LIST) if self.ik_use_waist else []
        ) + LEFT_JOINT_INDICES_LIST + RIGHT_JOINT_INDICES_LIST
        self.control_dim = len(self.control_joint_indices)

        self.motor_state = [MotorState() for _ in range(35)]
        self.lowstate_buffer = DataBuffer()
        self._last_q_target = np.zeros(self.control_dim, dtype=float)
        self.arms_enabled = False
        self.homing_active = False
        self.homing_reached = False
        self.homing_tolerance = 0.02
        self._last_left_goal_raw = None
        self._last_right_goal_raw = None
        self._goal_left_filt = None
        self._goal_right_filt = None
        self._reset_after_home = False
        self._initialized = False
        self._first_lowstate_logged = False
        self._last_homing_status_log_t = 0.0
        self._home_waist_target = np.zeros(3, dtype=float)

        self._T_off_right_static = self._mk_static_T(self._ee_off_right_xyz, self._ee_off_right_rpy_deg)
        self._T_off_left_static = self._mk_static_T(self._ee_off_left_xyz, self._ee_off_left_rpy_deg)
        self._T_off_right_auto = None
        self._T_off_left_auto = None
        self._auto_done_right = False
        self._auto_done_left = False

        self.get_logger().info("[startup] initializing IK solver...")
        self.ik_solver = G1IKSolver(
            debug=False,
            use_waist=self.ik_use_waist,
            max_dq_step=self.ik_max_dq_step,
        )
        self.get_logger().info("[startup] IK solver ready.")
        if hasattr(self.ik_solver, "set_orientation_mode"):
            self.ik_solver.set_orientation_mode(self.ik_orientation_mode)
        if hasattr(self.ik_solver, "set_use_waist"):
            self.ik_solver.set_use_waist(self.ik_use_waist)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.left_workspace_publisher = self.create_publisher(Marker, '/policypilot/workspace/left', 10)
        self.right_workspace_publisher = self.create_publisher(Marker, '/policypilot/workspace/right', 10)

        if not self.use_robot:
            self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(PoseStamped, "/policypilot/hand_goal/right", self._right_goal_callback, 10)
        self.create_subscription(PoseStamped, "/policypilot/hand_goal/left", self._left_goal_callback, 10)
        self.create_subscription(Bool, "/policypilot/arms/enabled", self._arms_controlled_callback, 10)
        self.create_subscription(Bool, "/policypilot/arms/home", self._homming_callback, 10)

        if self.use_robot:
            self.get_logger().info(f"[startup] initializing robot DDS interface on '{self.interface}'...")
            self._init_robot_interface()
            self.get_logger().info("[startup] robot DDS interface ready.")
        else:
            self.get_logger().info("[startup] simulation mode enabled (no robot DDS).")

        self._last_tick_time = None
        self.timer = self.create_timer(1.0 / self.rate_hz, self.main_loop)
        self.get_logger().info("[startup] arm_controller initialization complete.")

    def _mk_static_T(self, xyz, rpy_deg):
        """
        Create a fixed SE3 transform from XYZ translation and RPY rotation in degrees.

        Parameters
        ----------
        xyz : array-like of float
            Translation vector [x, y, z] in meters.
        rpy_deg : array-like of float
            Roll, pitch, yaw angles in degrees.

        Returns
        -------
        pinocchio.SE3
            Homogeneous transform combining translation and rotation.
        """

        rpy = np.radians(np.array(rpy_deg, dtype=float))
        R = pin.rpy.rpyToMatrix(rpy[0], rpy[1], rpy[2])
        return SE3(R, np.array(xyz, dtype=float))
    
    def _goal_error(self, side: str, T_goal: SE3):
        """
        Compute position and orientation error between current and target end-effector pose.

        Parameters
        ----------
        side : str
            Arm identifier ('left' or 'right').
        T_goal : SE3
            Desired end-effector target pose.

        Returns
        -------
        tuple(float, float)
            (position_error_m, orientation_error_rad)
        """

        M_cur = self._fk_current_ee(side)
        if M_cur is None or T_goal is None:
            return None, None
        dp = float(np.linalg.norm(T_goal.translation - M_cur.translation))
        dq = pin.Quaternion(M_cur.rotation.T @ T_goal.rotation)
        ang = 2.0 * math.atan2(
            math.sqrt(dq.x*dq.x + dq.y*dq.y + dq.z*dq.z),
            abs(dq.w)
        )
        return dp, ang

    def _lowpass_goal(self, T_prev: SE3, T_new: SE3, alpha: float) -> SE3:
        """
        Apply exponential smoothing between previous and new SE3 goals.

        Parameters
        ----------
        T_prev : SE3
            Previous goal pose.
        T_new : SE3
            New goal pose.
        alpha : float
            Low-pass coefficient (0.0–1.0).

        Returns
        -------
        SE3
            Smoothed goal transform.
        """

        if T_prev is None:
            return T_new
        p = (1.0 - alpha) * T_prev.translation + alpha * T_new.translation
        q0 = _mat_to_quat_wxyz(T_prev.rotation)
        q1 = _mat_to_quat_wxyz(T_new.rotation)
        qf = (1 - alpha) * q0 + alpha * q1
        qf = qf / np.linalg.norm(qf)
        Rf = _quat_wxyz_to_matrix(qf)
        return SE3(Rf, p)

    def _limit_ori_step(self, R_cur: np.ndarray, R_des: np.ndarray, max_step: float) -> np.ndarray:
        """
        Limit the angular step between current and desired rotation matrices.

        Parameters
        ----------
        R_cur : np.ndarray
            Current 3×3 rotation matrix.
        R_des : np.ndarray
            Desired 3×3 rotation matrix.
        max_step : float
            Maximum angular step (rad).

        Returns
        -------
        np.ndarray
            Limited rotation matrix.
        """

        R_err = R_cur.T @ R_des
        aa = pin.log3(R_err)
        nrm = float(np.linalg.norm(aa))
        if nrm <= 1e-12 or nrm <= max_step:
            return R_des
        aa_lim = aa * (max_step / nrm)
        return R_cur @ pin.exp3(aa_lim)

    def _fk_current_ee(self, side: str):
        """
        Compute the current end-effector pose (SE3) for the given arm using FK.

        Parameters
        ----------
        side : str
            'left' or 'right'.

        Returns
        -------
        SE3 or None
            Forward kinematics result for the end-effector.
        """

        try:
            q_full = pin.neutral(self.ik_solver.model)
            cur_all = self.get_current_motor_q() if self.use_robot else self._assemble_full_from_last()
            for jid_idx, ros_name in enumerate(self.ik_solver._ros_joint_names):
                if ros_name in self.ik_solver._name_to_q_index:
                    q_full[self.ik_solver._name_to_q_index[ros_name]] = float(cur_all[jid_idx])
            pin.forwardKinematics(self.ik_solver.model, self.ik_solver.data, q_full)
            pin.updateFramePlacements(self.ik_solver.model, self.ik_solver.data)
            fid = self.ik_solver._fid_right if side == 'right' else self.ik_solver._fid_left
            if fid is None:
                return None
            return self.ik_solver.data.oMf[fid]
        except Exception:
            return None

    def _gate_auto_calibration(self, T_goal_in: SE3, side: str):
        """
        Check if the current end-effector pose is close enough to the incoming goal
        to perform automatic end-effector calibration.

        Parameters
        ----------
        T_goal_in : SE3
            Incoming target pose before any static offset is applied.
        side : str
            Arm identifier ('left' or 'right').

        Returns
        -------
        SE3 or None
            Current SE3 pose if within calibration threshold, otherwise None.
        """

        M_cur = self._fk_current_ee(side)
        if M_cur is None:
            return None
        dp = np.linalg.norm(T_goal_in.translation - M_cur.translation)
        dq = pin.Quaternion(M_cur.rotation.T @ T_goal_in.rotation)
        ang = 2 * math.atan2(np.linalg.norm([dq.x, dq.y, dq.z]), abs(dq.w))
        if dp < 0.05 and ang < math.radians(12.0):
            return M_cur
        return None

    def _apply_offsets_and_filters(self, side: str, T_goal_input: SE3):
        """
        Apply static and auto-calibrated offsets to an incoming goal and filter it.

        This function handles end-effector calibration (static + automatic),
        goal smoothing, and orientation step limitation.

        Parameters
        ----------
        side : str
            Arm identifier ('left' or 'right').
        T_goal_input : SE3
            Raw goal transform.

        Returns
        -------
        SE3
            Adjusted and filtered goal for IK solver.
        """

        T_static = self._T_off_right_static if side == 'right' else self._T_off_left_static
        T_auto = self._T_off_right_auto if side == 'right' else self._T_off_left_auto
        auto_done = self._auto_done_right if side == 'right' else self._auto_done_left

        if self.ee_auto_calibrate and not auto_done:
            M_cur_ok = self._gate_auto_calibration(T_goal_input, side)
            if M_cur_ok is not None:
                T_pre = T_goal_input * T_static
                T_auto_new = T_pre.inverse() * M_cur_ok
                if side == 'right':
                    self._T_off_right_auto = T_auto_new; self._auto_done_right = True
                    t = T_auto_new.translation
                    self.get_logger().info(f"[IK] auto-calibrated right: d=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})")
                else:
                    self._T_off_left_auto = T_auto_new; self._auto_done_left = True
                    t = T_auto_new.translation
                    self.get_logger().info(f"[IK] auto-calibrated left: d=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})")
                T_auto = T_auto_new

        T_raw = T_goal_input * T_static * (T_auto if T_auto is not None else SE3.Identity())

        if side == 'right':
            self._goal_right_filt = self._lowpass_goal(self._goal_right_filt, T_raw, self.ik_goal_filter_alpha)
            T_use = self._goal_right_filt
        else:
            self._goal_left_filt = self._lowpass_goal(self._goal_left_filt, T_raw, self.ik_goal_filter_alpha)
            T_use = self._goal_left_filt

        M_cur = self._fk_current_ee(side)
        if (M_cur is not None) and (T_use is not None):
            R_lim = self._limit_ori_step(M_cur.rotation, T_use.rotation, self.ik_max_ori_step_rad)
            T_use = SE3(R_lim, T_use.translation.copy())

        return T_use

    def _init_robot_interface(self):
        """
        Initialize the DDS interface for real robot communication.

        Sets up Unitree DDS subscribers and publishers, initializes communication
        channels, waits for the first LowState message, and configures motor gains
        and modes for all arm joints.

        Attributes initialized
        ----------------------
        lowstate_subscriber : ChannelSubscriber
            Subscribes to the `rt/lowstate` DDS topic.
        lowcmd_publisher : ChannelPublisher
            Publishes LowCmd messages to the `rt/arm_sdk` DDS topic.
        subscribe_thread : threading.Thread
            Thread continuously reading DDS LowState data.
        msg : unitree_hg_msg_dds__LowCmd_
            Pre-allocated DDS command message structure.
        crc : CRC
            CRC calculator instance used for outgoing messages.
        """

        self.get_logger().info(f"[dds] ChannelFactoryInitialize(domain=0, iface={self.interface})")
        ChannelFactoryInitialize(0, self.interface)
        self.get_logger().info("[dds] ChannelFactoryInitialize done")

        self.lowstate_subscriber = ChannelSubscriber('rt/lowstate', LowState_)
        self.lowstate_subscriber.Init()
        self.get_logger().info("[dds] lowstate subscriber initialized (topic=rt/lowstate)")

        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state, daemon=True)
        self.subscribe_thread.start()
        self.get_logger().info("[dds] lowstate reader thread started")

        self.lowcmd_publisher = ChannelPublisher('rt/arm_sdk', LowCmd_)
        self.lowcmd_publisher.Init()
        self.get_logger().info("[dds] lowcmd publisher initialized (topic=rt/arm_sdk)")

        wait_t0 = time.time()
        last_log_t = wait_t0 - 1.0
        while not self.lowstate_buffer.GetData():
            now = time.time()
            if now - last_log_t >= 1.0:
                elapsed = now - wait_t0
                self.get_logger().info(
                    f"[dds] waiting for LowState data... {elapsed:.1f}s elapsed (iface={self.interface})"
                )
                last_log_t = now
            time.sleep(0.01)

        self.get_logger().info(f"[dds] LowState received after {time.time() - wait_t0:.2f}s")

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.all_motor_q = self.get_current_motor_q()

        # Match the official Unitree g1_arm7_sdk_dds example format.
        self.kp_arm = 60.0
        self.kd_arm = 1.5
        self.kp_waist = 60.0
        self.kd_waist = 1.5
        if self.ik_use_waist:
            for idx, jid in enumerate(WAIST_JOINT_INDICES_LIST):
                self.msg.motor_cmd[jid].q = float(self.all_motor_q[jid])
                self.msg.motor_cmd[jid].dq = 0.0
                self.msg.motor_cmd[jid].tau = 0.0
                self.msg.motor_cmd[jid].kp = self.kp_waist
                self.msg.motor_cmd[jid].kd = self.kd_waist
        for jid in G1_29_JointArmIndex:
            self.msg.motor_cmd[jid].q = float(self.all_motor_q[jid.value])
            self.msg.motor_cmd[jid].dq = 0.0
            self.msg.motor_cmd[jid].tau = 0.0
            self.msg.motor_cmd[jid].kp = self.kp_arm
            self.msg.motor_cmd[jid].kd = self.kd_arm

        try:
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 0.0
        except Exception:
            pass

        self.q_target = np.zeros(self.control_dim)
        self.tauff_target = np.zeros(self.control_dim)
        self._initialized = True
        self.get_logger().info("[dds] arm command structures initialized using official arm_sdk format")

    def _subscribe_motor_state(self):
        """
        Thread loop continuously reading motor state messages from DDS.

        Runs as a daemon thread to asynchronously update internal motor states
        and store the latest LowState message in `lowstate_buffer`.

        Notes
        -----
        - This method blocks indefinitely while ROS 2 is running.
        - Updates both joint positions and velocities for each motor.
        """

        while rclpy.ok():
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                if not self._first_lowstate_logged:
                    self._first_lowstate_logged = True
                    self.get_logger().info(
                        f"[dds] first lowstate packet received (mode_machine={getattr(msg, 'mode_machine', 'n/a')})"
                    )
                self.lowstate_buffer.SetData(msg)
                for i in range(len(self.motor_state)):
                    self.motor_state[i].q  = msg.motor_state[i].q
                    self.motor_state[i].dq = msg.motor_state[i].dq
            time.sleep(0.001)

    def get_mode_machine(self) -> int:
        """
        Get the current robot mode from the latest LowState message.

        Returns
        -------
        int
            Current machine mode identifier, or 0 if no data available.
        """

        msg = self.lowstate_buffer.GetData()
        return getattr(msg, "mode_machine", 0) if msg is not None else 0

    def get_current_motor_q(self) -> np.ndarray:
        """
        Retrieve the current joint positions (q) for all robot joints.

        Returns
        -------
        np.ndarray
            Array of joint positions (rad) ordered according to `G1_29_JointIndex`.
        """

        msg = self.lowstate_buffer.GetData()
        return np.array([msg.motor_state[id].q for id in G1_29_JointIndex], dtype=float)

    def _compose_control_target(self, left, right, waist=None) -> np.ndarray:
        chunks = []
        if self.ik_use_waist:
            if waist is None:
                waist = np.zeros(3, dtype=float)
            chunks.append(np.asarray(waist, dtype=float))
        chunks.append(np.asarray(left, dtype=float))
        chunks.append(np.asarray(right, dtype=float))
        return np.concatenate(chunks)

    def _split_control_target(self, q_target: np.ndarray):
        q_arr = np.asarray(q_target, dtype=float)
        out = {}
        offset = 0
        if self.ik_use_waist:
            out["waist"] = q_arr[:3].copy()
            offset = 3
        out["left"] = q_arr[offset:offset + 7].copy()
        out["right"] = q_arr[offset + 7:offset + 14].copy()
        return out

    def _assemble_full_from_last(self) -> np.ndarray:
        """
        Build a complete 29-DOF joint configuration vector from the latest arm targets.

        Combines left and right 7-DOF joint targets into a full-body vector.

        Returns
        -------
        np.ndarray
            Full 29-element joint configuration array.
        """

        full = np.zeros(29, dtype=float)
        parts = self._split_control_target(self._last_q_target)
        if self.ik_use_waist:
            for i, jidx in enumerate(WAIST_JOINT_INDICES_LIST):
                full[jidx] = parts["waist"][i]
        for i, jidx in enumerate(LEFT_JOINT_INDICES_LIST):
            full[jidx] = parts["left"][i]
        for i, jidx in enumerate(RIGHT_JOINT_INDICES_LIST):
            full[jidx] = parts["right"][i]
        return full

    def _current_arm_q(self) -> np.ndarray:
        """
        Return the measured 14-DOF arm joint vector in left-then-right order.

        In simulation mode, fall back to the last commanded target.
        """

        if not self.use_robot:
            parts = self._split_control_target(self._last_q_target)
            return np.concatenate((parts["left"], parts["right"]))

        cur = self.get_current_motor_q()
        left = [cur[j] for j in LEFT_JOINT_INDICES_LIST]
        right = [cur[j] for j in RIGHT_JOINT_INDICES_LIST]
        return np.array(left + right, dtype=float)

    def _current_waist_q(self) -> np.ndarray:
        """Return the measured 3-DOF waist joint vector."""

        if not self.ik_use_waist:
            return np.zeros(3, dtype=float)
        if not self.use_robot:
            return self._split_control_target(self._last_q_target).get("waist", np.zeros(3, dtype=float))

        cur = self.get_current_motor_q()
        return np.array([cur[j] for j in WAIST_JOINT_INDICES_LIST], dtype=float)

    def _current_control_q(self) -> np.ndarray:
        """Return the measured control vector in waist-left-right layout."""

        if self.ik_use_waist:
            arm_q = self._current_arm_q()
            return self._compose_control_target(
                left=arm_q[:7],
                right=arm_q[7:],
                waist=self._current_waist_q(),
            )
        return self._current_arm_q()

    def _hold_non_arm_joints(self):
        """
        Release arm_sdk ownership while arm control is disabled.

        Notes
        -----
        - Does nothing in simulation mode (`use_robot=False`).
        """

        if not self.use_robot:
            return
        try:
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 0.0
        except Exception:
            return
        self.msg.crc = self.crc.Crc(self.msg)
        self.lowcmd_publisher.Write(self.msg)

    def _arms_controlled_callback(self, msg: Bool):
        """
        ROS 2 callback to enable or disable arm control.

        Parameters
        ----------
        msg : std_msgs.msg.Bool
            True to enable arm control, False to disable.

        Behavior
        --------
        - On enable: stores current joint state as last target.
        - On disable: stops sending active motion commands.
        """

        self.arms_enabled = msg.data
        if self.arms_enabled:
            try:
                self._last_q_target = self._current_control_q()
            except Exception:
                pass
            self.get_logger().info("Arm ENABLED.")
        else:
            if self.use_robot:
                self._hold_non_arm_joints()
            self.get_logger().info("Arm DISABLED")

    def _homming_callback(self, msg: Bool):
        """
        ROS 2 callback that triggers the homing sequence for both arms.

        Parameters
        ----------
        msg : std_msgs.msg.Bool
            If True, initiates homing to predefined joint targets.

        Notes
        -----
        - Clears IK goals before starting the homing motion.
        - Sets internal flags for homing management.
        """

        if msg.data:
            self.get_logger().info("Moving both arms to HOME position.")
            self.homing_active = True
            self.homing_reached = False
            self._reset_after_home = False
            if self.ik_use_waist:
                try:
                    self._home_waist_target = self._current_waist_q()
                except Exception:
                    self._home_waist_target = np.zeros(3, dtype=float)
            if hasattr(self.ik_solver, "clear_goals"):
                self.ik_solver.clear_goals()

    def _transform_pose_to_world(self, ps: PoseStamped) -> PoseStamped:
        """
        Transform an incoming pose message into the world (IK) reference frame.

        Parameters
        ----------
        ps : geometry_msgs.msg.PoseStamped
            Input pose message with arbitrary frame_id.

        Returns
        -------
        geometry_msgs.msg.PoseStamped
            Pose transformed into `self.frame` if possible; original if TF lookup fails.
        """

        if not ps.header.frame_id or ps.header.frame_id == self.frame:
            return ps
        try:
            tf = self.tf_buffer.lookup_transform(self.frame, ps.header.frame_id, Time(), timeout=Duration(seconds=0.2))
            return do_transform_pose(ps, tf)
        except Exception as e:
            self.get_logger().warning(f"[IK] TF {ps.header.frame_id}->{self.frame} failed: {e}")
            return ps

    def _right_goal_callback(self, msg: PoseStamped):
        """
        ROS 2 callback for the right-hand end-effector goal.

        Parameters
        ----------
        msg : geometry_msgs.msg.PoseStamped
            Desired right-hand pose (can be in any TF frame).

        Behavior
        --------
        - Applies TF transformation to the world frame.
        - Handles homing reset alignment if needed.
        - Applies static and auto-calibration offsets.
        - Updates the IK solver's right-hand goal.
        """

        if self.homing_active:
            return
        
        if not self.arms_enabled:
            return

        if self._reset_after_home:
            self._reset_after_home = False
            self.homing_reached = False
            try:
                self._last_q_target = self._current_control_q()
            except Exception:
                self._last_q_target = self._compose_control_target(
                    self.home_left,
                    self.home_right,
                    waist=self._home_waist_target,
                )

            self._goal_left_filt  = None
            self._goal_right_filt = None

            self.ik_solver.set_current_configuration(self._split_control_target(self._last_q_target))

        msg_tf = self._transform_pose_to_world(msg)
        o, p = msg_tf.pose.orientation, msg_tf.pose.position
        q = pin.Quaternion(o.w, o.x, o.y, o.z)
        T_goal_in = SE3(q.matrix(), np.array([p.x, p.y, p.z]))

        self._last_right_goal_raw = T_goal_in
        T_goal_use = self._apply_offsets_and_filters('right', T_goal_in)
        if T_goal_use is not None:
            self.ik_solver.set_goal("right", T_goal_use)


    def _left_goal_callback(self, msg: PoseStamped):
        """
        ROS 2 callback for the left-hand end-effector goal.

        Parameters
        ----------
        msg : geometry_msgs.msg.PoseStamped
            Desired left-hand pose (can be in any TF frame).

        Behavior
        --------
        - Applies TF transformation to the world frame.
        - Handles homing reset alignment if needed.
        - Applies static and auto-calibration offsets.
        - Updates the IK solver's left-hand goal.
        """

        if self.homing_active:
            return
        
        if not self.arms_enabled:
            return

        if self._reset_after_home:
            self._reset_after_home = False
            self.homing_reached = False
            try:
                self._last_q_target = self._current_control_q()
            except Exception:
                self._last_q_target = self._compose_control_target(
                    self.home_left,
                    self.home_right,
                    waist=self._home_waist_target,
                )

            self._goal_left_filt  = None
            self._goal_right_filt = None

            self.ik_solver.set_current_configuration(self._split_control_target(self._last_q_target))

        msg_tf = self._transform_pose_to_world(msg)
        o, p = msg_tf.pose.orientation, msg_tf.pose.position
        q = pin.Quaternion(o.w, o.x, o.y, o.z)
        T_goal_in = SE3(q.matrix(), np.array([p.x, p.y, p.z]))

        self._last_left_goal_raw = T_goal_in
        T_goal_use = self._apply_offsets_and_filters('left', T_goal_in)
        if T_goal_use is not None:
            self.ik_solver.set_goal("left", T_goal_use)


    def _compute_dt(self) -> float:
        """
        Compute the elapsed time (Δt) between consecutive main loop cycles.

        Returns
        -------
        float
            Time difference in seconds (clamped to [1e-4, 0.1]).
        """

        now = time.time()
        if self._last_tick_time is None:
            dt = 1.0 / self.rate_hz
        else:
            dt = max(1e-4, min(0.1, now - self._last_tick_time))
        self._last_tick_time = now
        return dt
    
    def _publish_workspace(self, arm):
        marker = Marker()
        marker.header.frame_id = WORKSPACE["frame"]
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "workspace"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.005  
        marker.color = ColorRGBA(r=0.1, g=1.0, b=0.3, a=0.9)

        points = WORKSPACE[arm]

        pts = {k: Point(x=v[0], y=v[1], z=v[2]) for k, v in points.items()}

        edges = [
            # Bottom rectangle
            ("left_bottom_front", "right_bottom_front"),
            ("right_bottom_front", "right_bottom_back"),
            ("right_bottom_back", "left_bottom_back"),
            ("left_bottom_back", "left_bottom_front"),

            # Top rectangle
            ("left_top_front", "right_top_front"),
            ("right_top_front", "right_top_back"),
            ("right_top_back", "left_top_back"),
            ("left_top_back", "left_top_front"),

            # Vertical edges
            ("left_bottom_front", "left_top_front"),
            ("right_bottom_front", "right_top_front"),
            ("left_bottom_back", "left_top_back"),
            ("right_bottom_back", "right_top_back"),
        ]

        for a, b in edges:
            marker.points.append(pts[a])
            marker.points.append(pts[b])

        if arm == "left_arm":
            self.left_workspace_publisher.publish(marker)
        else:
            self.right_workspace_publisher.publish(marker)

    def main_loop(self):
        """
        Main control loop executed at `rate_hz` frequency.

        Core Responsibilities
        ---------------------
        - Update LowState data from DDS.
        - Hold non-arm joints when arms are disabled.
        - Execute homing sequence if active.
        - Update IK solver configuration and compute joint targets.
        - Apply velocity and smoothing limits.
        - Publish joint targets via DDS or /joint_states (simulation).

        Notes
        -----
        - The loop manages both autonomous IK motion and homing control.
        - It automatically synchronizes the IK goals when returning to home.
        """

        self._publish_workspace("left_arm")
        self._publish_workspace("right_arm")

        if not getattr(self, "_initialized", False):
            return
        
        if self.use_robot:
            robot_data = self.lowstate_subscriber.Read()
            if robot_data is not None:
                self.lowstate_buffer.SetData(robot_data)
                for i in range(len(self.motor_state)):
                    self.motor_state[i].q  = robot_data.motor_state[i].q
                    self.motor_state[i].dq = robot_data.motor_state[i].dq

        if not self.arms_enabled:
            self._hold_non_arm_joints()
            return

        if self.homing_active:
            q_target = self._compose_control_target(
                self.home_left,
                self.home_right,
                waist=self._home_waist_target,
            )
            q_measured = self._current_control_q()
            homing_error = float(np.linalg.norm(q_target - q_measured))
            if homing_error < self.homing_tolerance:

                self.homing_active = False
                self.homing_reached = True
                self._last_q_target = q_target.copy()

                if hasattr(self.ik_solver, "clear_goals"):
                    self.ik_solver.clear_goals()

                self.ik_solver.set_current_configuration(
                    self._split_control_target(q_target)
                )

                try:
                    q_full = pin.neutral(self.ik_solver.model)
                    if self.ik_use_waist:
                        for i, waist_i in enumerate(WAIST_JOINT_INDICES_LIST):
                            q_full[
                                self.ik_solver._name_to_q_index[self.ik_solver._ros_joint_names[waist_i]]
                            ] = self._home_waist_target[i]
                    for i, arm_i in enumerate(LEFT_JOINT_INDICES_LIST):
                        q_full[self.ik_solver._name_to_q_index[self.ik_solver._ros_joint_names[arm_i]]] = self.home_left[i]
                    for i, arm_i in enumerate(RIGHT_JOINT_INDICES_LIST):
                        q_full[self.ik_solver._name_to_q_index[self.ik_solver._ros_joint_names[arm_i]]] = self.home_right[i]

                    pin.forwardKinematics(self.ik_solver.model, self.ik_solver.data, q_full)
                    pin.updateFramePlacements(self.ik_solver.model, self.ik_solver.data)

                    T_left  = self.ik_solver.data.oMf[self.ik_solver._fid_left]
                    T_right = self.ik_solver.data.oMf[self.ik_solver._fid_right]

                    self._goal_left_filt  = T_left.copy()
                    self._goal_right_filt = T_right.copy()
                    if hasattr(self.ik_solver, "set_goal"):
                        self.ik_solver.set_goal("left",  T_left.copy())
                        self.ik_solver.set_goal("right", T_right.copy())

                    self._reset_after_home = True

                    self.get_logger().info("IK solver goals aligned with home pose.")
                except Exception as e:
                    self.get_logger().warning(f"Failed to align IK goals with home: {e}")

                self.get_logger().info("Home position reached.")
            elif self.use_robot:
                now = time.time()
                if now - self._last_homing_status_log_t >= 1.0:
                    self._last_homing_status_log_t = now
                    self.get_logger().warn(
                        f"[homing] waiting for measured arm motion, current error={homing_error:.3f} rad"
                    )

        elif self.homing_reached:
            q_target = self._compose_control_target(
                self.home_left,
                self.home_right,
                waist=self._home_waist_target,
            )

        else:
            current_all = self.get_current_motor_q() if self.use_robot else self._assemble_full_from_last()

            try:
                self.ik_solver.set_current_configuration(self._split_control_target(self._last_q_target))
            except Exception:
                pass

            if self._goal_left_filt is not None:
                self.ik_solver.set_goal("left", self._goal_left_filt)
            if self._goal_right_filt is not None:
                self.ik_solver.set_goal("right", self._goal_right_filt)

            q_dict = self.ik_solver.get_joint_targets(current_all)
            q_target = self._last_q_target.copy()
            offset = 0
            if self.ik_use_waist and "waist" in q_dict:
                q_target[0:3] = q_dict["waist"]
                offset = 3
            if "left" in q_dict:
                q_target[offset:offset + 7] = q_dict["left"]
            if "right" in q_dict:
                q_target[offset + 7:offset + 14] = q_dict["right"]

        dt = self._compute_dt()
        max_step = self.arm_velocity_limit * dt
        dq = np.clip(q_target - self._last_q_target, -max_step, max_step)

        q_unsmoothed = self._last_q_target + dq
        q_smooth = (1.0 - self.ik_alpha) * self._last_q_target + self.ik_alpha * q_unsmoothed
        self._last_q_target = q_smooth.copy()

        if self.use_robot:
            try:
                self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0
            except Exception:
                pass

            arm_offset = 0
            if self.ik_use_waist:
                for idx, joint_idx in enumerate(WAIST_JOINT_INDICES_LIST):
                    self.msg.motor_cmd[joint_idx].q = float(q_smooth[idx])
                    self.msg.motor_cmd[joint_idx].dq = 0.0
                    self.msg.motor_cmd[joint_idx].tau = 0.0
                    self.msg.motor_cmd[joint_idx].kp = self.kp_waist
                    self.msg.motor_cmd[joint_idx].kd = self.kd_waist
                arm_offset = 3

            for idx, jid in enumerate(G1_29_JointArmIndex):
                self.msg.motor_cmd[jid].q   = float(q_smooth[arm_offset + idx])
                self.msg.motor_cmd[jid].dq  = 0.0
                self.msg.motor_cmd[jid].tau = float(0.0)
                self.msg.motor_cmd[jid].kp = self.kp_arm
                self.msg.motor_cmd[jid].kd = self.kd_arm

            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)
        else:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [JOINT_NAMES_ROS[i] for i in sorted(JOINT_NAMES_ROS.keys())]
            js.position = [0.0] * len(js.name)
            offset = 0
            if self.ik_use_waist:
                for idx, joint_idx in enumerate(WAIST_JOINT_INDICES_LIST):
                    js.position[joint_idx] = float(q_smooth[idx])
                offset = 3
            for idx, joint_idx in enumerate(LEFT_JOINT_INDICES_LIST):
                js.position[joint_idx] = float(q_smooth[offset + idx])
            for idx, joint_idx in enumerate(RIGHT_JOINT_INDICES_LIST):
                js.position[joint_idx] = float(q_smooth[offset + 7 + idx])
            self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
