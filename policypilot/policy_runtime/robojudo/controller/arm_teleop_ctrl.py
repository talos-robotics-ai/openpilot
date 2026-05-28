import logging
import sys
import threading
import time
from contextlib import suppress
from pathlib import Path

import numpy as np
import pinocchio as pin

from robojudo.controller import Controller, ctrl_registry
from robojudo.controller.ctrl_cfgs import ArmTeleopCtrlCfg

logger = logging.getLogger(__name__)


def _ensure_teleop_path():
    controlboard_root = Path(__file__).resolve().parents[3]
    teleop_root = controlboard_root / "teleop"
    teleop_root_str = teleop_root.as_posix()
    if teleop_root.exists() and teleop_root_str not in sys.path:
        sys.path.insert(0, teleop_root_str)
    return teleop_root


def _ensure_teleimager_path():
    controlboard_root = Path(__file__).resolve().parents[3]
    teleimager_src = controlboard_root.parent / "xr_teleoperate" / "teleop" / "teleimager" / "src"
    teleimager_src_str = teleimager_src.as_posix()
    if teleimager_src.exists() and teleimager_src_str not in sys.path:
        sys.path.insert(0, teleimager_src_str)
    return teleimager_src


def _fk_wrist_targets(arm_ik, q):
    model = arm_ik.reduced_robot.model
    data = model.createData()
    pin.framesForwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    left_pose = np.array(data.oMf[arm_ik.L_hand_id].homogeneous)
    right_pose = np.array(data.oMf[arm_ik.R_hand_id].homogeneous)
    return left_pose, right_pose


def _needs_image_transport(exc: Exception) -> bool:
    msg = str(exc)
    return "requires zmq=True or webrtc=True" in msg


def _make_default_wrist_pose(y_offset: float) -> np.ndarray:
    pose = np.eye(4, dtype=np.float64)
    pose[0, 3] = 0.25
    pose[1, 3] = y_offset
    pose[2, 3] = 0.08
    return pose


G1_LEFT_ARM_MOTOR_NAMES = [
    "kLeftShoulderPitch",
    "kLeftShoulderRoll",
    "kLeftShoulderYaw",
    "kLeftElbow",
    "kLeftWristRoll",
    "kLeftWristPitch",
    "kLeftWristYaw",
]

G1_RIGHT_ARM_MOTOR_NAMES = [
    "kRightShoulderPitch",
    "kRightShoulderRoll",
    "kRightShoulderYaw",
    "kRightElbow",
    "kRightWristRoll",
    "kRightWristPitch",
    "kRightWristYaw",
]

G1_LEFT_INSPIRE_MOTOR_NAMES = [
    "kLeftHandPinky",
    "kLeftHandRing",
    "kLeftHandMiddle",
    "kLeftHandIndex",
    "kLeftHandThumbBend",
    "kLeftHandThumbRotation",
]

G1_RIGHT_INSPIRE_MOTOR_NAMES = [
    "kRightHandPinky",
    "kRightHandRing",
    "kRightHandMiddle",
    "kRightHandIndex",
    "kRightHandThumbBend",
    "kRightHandThumbRotation",
]


def _parse_6d_list(values):
    if not isinstance(values, list) or len(values) != 6:
        return None
    try:
        return [float(v) for v in values]
    except (TypeError, ValueError):
        return None


def _copy_hand_packet(packet: dict) -> dict:
    return {
        "left_state": list(packet.get("left_state", [])),
        "right_state": list(packet.get("right_state", [])),
        "left_action": list(packet.get("left_action", [])),
        "right_action": list(packet.get("right_action", [])),
    }


@ctrl_registry.register
class ArmTeleopCtrl(Controller):
    cfg_ctrl: ArmTeleopCtrlCfg

    ARM_JOINT_NAMES = [
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ]

    def __init__(self, cfg_ctrl: ArmTeleopCtrlCfg, env=None, device="cpu"):
        super().__init__(cfg_ctrl=cfg_ctrl, env=env, device=device)
        if env is None or not hasattr(env, "set_joint_override_targets"):
            raise ValueError("ArmTeleopCtrl requires a UnitreeEnv-like environment with joint override support.")

        _ensure_teleop_path()
        from teleop.robot_control.robot_arm_ik import G1_29_ArmIK

        self.arm_ik = G1_29_ArmIK()
        self.tv_wrapper = None
        self.img_client = None
        self.wrist_zmq = None
        self.wrist_zmq_ctx = None
        self.wrist_zmq_sub = None
        self.hand_zmq = None
        self.hand_zmq_ctx = None
        self.hand_zmq_sub = None
        self.recorder = None
        self._preview_enabled = False
        self._warned_bad_wrist_packet = False
        self._warned_legacy_right_only = False
        self._warned_no_source = False
        self._warned_waiting_for_input = False
        self._warned_bad_hand_packet = False
        self._warned_missing_hand_packet = False
        self._last_record_backpressure_log_t = 0.0
        self._last_ctrl_data = {"source": "arm_teleop", "active": False}
        self._stop_event = threading.Event()
        self._manual_enabled = threading.Event()
        self._manual_listener_thread = None
        self._manual_listener_stop = None
        self._record_thread = None
        self._record_lock = threading.Lock()
        self._last_live_input_t = 0.0
        self._last_ik_solve_t = 0.0
        self._ik_dt = 1.0 / max(self.cfg_ctrl.solve_frequency, 1.0)
        self._default_left_wrist_pose = _make_default_wrist_pose(0.15)
        self._default_right_wrist_pose = _make_default_wrist_pose(-0.15)
        self._record_toggle_requested = False
        self._record_running = False
        self._record_dt = 1.0 / max(self.cfg_ctrl.record_frequency, 1.0)
        self._camera_config = {
            "head_camera": {
                "enable_zmq": False,
                "enable_webrtc": False,
                "binocular": False,
                "image_shape": [0, 0, 0],
                "webrtc_port": 0,
            },
            "left_wrist_camera": {"enable_zmq": False},
            "right_wrist_camera": {"enable_zmq": False},
        }
        self._latest_hand_packet = {
            "left_state": [0.0] * 6,
            "right_state": [0.0] * 6,
            "left_action": [0.0] * 6,
            "right_action": [0.0] * 6,
        }
        self._got_hand_packet = False

        if not self.cfg_ctrl.manual_enable:
            self._manual_enabled.set()

        current_arm_q, _ = self.env.get_current_joint_state(self.ARM_JOINT_NAMES)
        self.left_wrist_pose, self.right_wrist_pose = _fk_wrist_targets(self.arm_ik, current_arm_q)
        self._last_sol_q = current_arm_q.copy()
        self._last_sol_tauff = np.zeros_like(current_arm_q)
        self._record_snapshot = {
            "arm_q": current_arm_q.copy(),
            "arm_action": current_arm_q.copy(),
            "hand_packet": self._latest_hand_packet.copy(),
            "got_hand_packet": False,
        }

        if self.cfg_ctrl.wrist_zmq:
            import zmq

            self.wrist_zmq = zmq
            self.wrist_zmq_ctx = zmq.Context()
            self.wrist_zmq_sub = self.wrist_zmq_ctx.socket(zmq.SUB)
            self.wrist_zmq_sub.setsockopt(zmq.CONFLATE, 1)
            self.wrist_zmq_sub.connect(f"tcp://{self.cfg_ctrl.wrist_zmq_host}:{self.cfg_ctrl.wrist_zmq_port}")
            self.wrist_zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "")
            logger.info(
                "ArmTeleopCtrl using wrist ZMQ source tcp://%s:%s",
                self.cfg_ctrl.wrist_zmq_host,
                self.cfg_ctrl.wrist_zmq_port,
            )
        try:
            from televuer import TeleVuerWrapper
        except Exception as exc:
            if not self.cfg_ctrl.wrist_zmq:
                raise ImportError(
                    "ArmTeleopCtrl could not import TeleVuerWrapper. "
                    "Install the XR teleop stack or enable wrist_zmq in the controller config."
                ) from exc
            logger.warning("ArmTeleopCtrl TeleVuer unavailable, falling back to wrist ZMQ only: %s", exc)
        else:
            tv_kwargs = dict(
                use_hand_tracking=self.cfg_ctrl.input_mode == "hand",
                binocular=False,
                img_shape=[720, 1280, 3],
                zmq=False,
                webrtc=False,
                webrtc_url="",
            )
            effective_display_mode = self.cfg_ctrl.display_mode
            try:
                self.tv_wrapper = TeleVuerWrapper(
                    display_mode=effective_display_mode,
                    **tv_kwargs,
                )
            except ValueError as exc:
                if _needs_image_transport(exc):
                    logger.warning(
                        "ArmTeleopCtrl requested display_mode=%s without image transport. "
                        "Retrying TeleVuer in pass-through mode.",
                        effective_display_mode,
                    )
                    effective_display_mode = "pass-through"
                    try:
                        self.tv_wrapper = TeleVuerWrapper(
                            display_mode=effective_display_mode,
                            **tv_kwargs,
                        )
                    except Exception as retry_exc:
                        if self.cfg_ctrl.wrist_zmq:
                            logger.warning(
                                "ArmTeleopCtrl TeleVuer startup failed after pass-through retry; "
                                "continuing with wrist ZMQ only: %s",
                                retry_exc,
                            )
                            self.tv_wrapper = None
                        else:
                            raise retry_exc
                elif self.cfg_ctrl.wrist_zmq:
                    logger.warning("ArmTeleopCtrl TeleVuer startup failed, continuing with wrist ZMQ only: %s", exc)
                    self.tv_wrapper = None
                else:
                    raise

            if self.tv_wrapper is not None:
                logger.info(
                    "ArmTeleopCtrl using TeleVuer input mode=%s display_mode=%s",
                    self.cfg_ctrl.input_mode,
                    effective_display_mode,
                )

        need_image_client = self.cfg_ctrl.render_head_image or self.cfg_ctrl.record
        if need_image_client:
            try:
                _ensure_teleimager_path()
                from teleimager.image_client import ImageClient

                self.img_client = ImageClient(host=self.cfg_ctrl.img_server_ip)
                self._camera_config = self.img_client.get_cam_config()
                self._preview_enabled = self.tv_wrapper is not None and self.cfg_ctrl.render_head_image
            except Exception as exc:
                logger.warning("ArmTeleopCtrl could not start ImageClient: %s", exc)

        if self.cfg_ctrl.record:
            self._setup_recording()
            self._setup_hand_zmq()
            if self.recorder is not None:
                self._record_thread = threading.Thread(target=self._record_loop, daemon=True, name="arm-teleop-recorder")
                self._record_thread.start()

        self._start_manual_listener()
        self.thread = threading.Thread(target=self._run_loop, daemon=True, name="arm-teleop")
        self.thread.start()

    def reset(self):
        self._last_ctrl_data = {"source": "arm_teleop", "active": False}

    def get_data(self):
        return self._last_ctrl_data

    def post_step_callback(self, commands: list[str] | None = None):
        commands = commands or []
        if "[SHUTDOWN]" in commands:
            self.close()

    def close(self):
        if self._stop_event.is_set():
            return
        self._stop_event.set()
        if self._manual_listener_stop is not None:
            with suppress(Exception):
                self._manual_listener_stop()
        if self._manual_listener_thread is not None and self._manual_listener_thread.is_alive():
            self._manual_listener_thread.join(timeout=0.5)
        if self._record_thread is not None and self._record_thread.is_alive():
            self._record_thread.join(timeout=0.5)
        if self.thread.is_alive():
            self.thread.join(timeout=0.5)
        try:
            self.env.clear_joint_override_targets(self.ARM_JOINT_NAMES)
        except Exception:
            pass
        if self.tv_wrapper is not None:
            try:
                self.tv_wrapper.close()
            except Exception:
                pass
        if self.img_client is not None:
            try:
                self.img_client.close()
            except Exception:
                pass
        if self.wrist_zmq_sub is not None:
            try:
                self.wrist_zmq_sub.close()
            except Exception:
                pass
        if self.wrist_zmq_ctx is not None:
            try:
                self.wrist_zmq_ctx.term()
            except Exception:
                pass
        if self.hand_zmq_sub is not None:
            try:
                self.hand_zmq_sub.close()
            except Exception:
                pass
        if self.hand_zmq_ctx is not None:
            try:
                self.hand_zmq_ctx.term()
            except Exception:
                pass
        if self.recorder is not None:
            try:
                if self._record_running:
                    self.recorder.save_episode()
                    self._record_running = False
                self.recorder.close()
            except Exception:
                pass

    def _update_from_televuer(self):
        tele_data = self.tv_wrapper.get_tele_data()
        if tele_data is None:
            return False

        left_pose = np.asarray(tele_data.left_wrist_pose, dtype=np.float64)
        right_pose = np.asarray(tele_data.right_wrist_pose, dtype=np.float64)
        teleop_live = self._televuer_pose_is_live(left_pose, right_pose)
        if teleop_live:
            self.left_wrist_pose = left_pose
            self.right_wrist_pose = right_pose

        if self._preview_enabled and self.img_client is not None:
            try:
                head_img, _ = self.img_client.get_head_frame()
                self.tv_wrapper.render_to_xr(head_img)
            except Exception:
                self._preview_enabled = False

        return teleop_live

    def _update_from_wrist_zmq(self):
        try:
            raw = self.wrist_zmq_sub.recv(flags=self.wrist_zmq.NOBLOCK)
        except self.wrist_zmq.Again:
            return False

        wrist_flat = np.frombuffer(raw, dtype=np.float64)
        if wrist_flat.size == 32:
            wrist_pair = wrist_flat.reshape(2, 4, 4)
            self.left_wrist_pose = wrist_pair[0]
            self.right_wrist_pose = wrist_pair[1]
            return True

        if wrist_flat.size == 16:
            self.right_wrist_pose = wrist_flat.reshape(4, 4)
            if not self._warned_legacy_right_only:
                logger.warning(
                    "ArmTeleopCtrl received legacy 16-float wrist packet; only right wrist IK target is updated."
                )
                self._warned_legacy_right_only = True
            return True

        if not self._warned_bad_wrist_packet:
            logger.warning("ArmTeleopCtrl expected 32 float64 wrist values, got %s", wrist_flat.size)
            self._warned_bad_wrist_packet = True
        return False

    def _televuer_pose_is_live(self, left_pose: np.ndarray, right_pose: np.ndarray) -> bool:
        if left_pose.shape != (4, 4) or right_pose.shape != (4, 4):
            return False
        if not np.all(np.isfinite(left_pose)) or not np.all(np.isfinite(right_pose)):
            return False
        if np.allclose(left_pose, self._default_left_wrist_pose, atol=1e-4) and np.allclose(
            right_pose,
            self._default_right_wrist_pose,
            atol=1e-4,
        ):
            return False
        return True

    def _use_inspire_hand_stream(self) -> bool:
        return self.cfg_ctrl.ee in {"inspire_dfx", "inspire_ftp"}

    def _setup_recording(self):
        try:
            from teleop.utils.episode_writer import EpisodeWriter
        except Exception as exc:
            logger.warning("ArmTeleopCtrl could not import EpisodeWriter; recording disabled: %s", exc)
            self.recorder = None
            return

        task_dir = str(Path(self.cfg_ctrl.task_dir).expanduser() / self.cfg_ctrl.task_name)
        try:
            self.recorder = EpisodeWriter(
                task_dir=task_dir,
                task_goal=self.cfg_ctrl.task_goal,
                task_desc=self.cfg_ctrl.task_desc,
                task_steps=self.cfg_ctrl.task_steps,
                frequency=self.cfg_ctrl.record_frequency,
                rerun_log=self.cfg_ctrl.record_rerun_log,
            )
        except Exception as exc:
            logger.warning("ArmTeleopCtrl EpisodeWriter init failed with rerun_log=%s: %s. Retrying with rerun_log=False.",
                           self.cfg_ctrl.record_rerun_log, exc)
            self.recorder = EpisodeWriter(
                task_dir=task_dir,
                task_goal=self.cfg_ctrl.task_goal,
                task_desc=self.cfg_ctrl.task_desc,
                task_steps=self.cfg_ctrl.task_steps,
                frequency=self.cfg_ctrl.record_frequency,
                rerun_log=False,
            )

        self.recorder.info["joint_names"]["left_arm"] = G1_LEFT_ARM_MOTOR_NAMES
        self.recorder.info["joint_names"]["right_arm"] = G1_RIGHT_ARM_MOTOR_NAMES
        if self._use_inspire_hand_stream():
            self.recorder.info["joint_names"]["left_ee"] = G1_LEFT_INSPIRE_MOTOR_NAMES
            self.recorder.info["joint_names"]["right_ee"] = G1_RIGHT_INSPIRE_MOTOR_NAMES

    def _setup_hand_zmq(self):
        if self.recorder is None:
            return
        if not self._use_inspire_hand_stream():
            return

        import zmq

        self.hand_zmq = zmq
        self.hand_zmq_ctx = zmq.Context()
        self.hand_zmq_sub = self.hand_zmq_ctx.socket(zmq.SUB)
        self.hand_zmq_sub.setsockopt(zmq.CONFLATE, 1)
        self.hand_zmq_sub.connect(f"tcp://{self.cfg_ctrl.hand_zmq_host}:{self.cfg_ctrl.hand_zmq_port}")
        self.hand_zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        logger.info(
            "ArmTeleopCtrl using Inspire hand ZMQ source tcp://%s:%s",
            self.cfg_ctrl.hand_zmq_host,
            self.cfg_ctrl.hand_zmq_port,
        )

    def _update_from_hand_zmq(self):
        if self.hand_zmq_sub is None:
            return False

        try:
            msg = self.hand_zmq_sub.recv_json(flags=self.hand_zmq.NOBLOCK)
        except self.hand_zmq.Again:
            return False

        left_state = _parse_6d_list(msg.get("left_state"))
        right_state = _parse_6d_list(msg.get("right_state"))
        left_action = _parse_6d_list(msg.get("left_action"))
        right_action = _parse_6d_list(msg.get("right_action"))
        if left_state and right_state and left_action and right_action:
            hand_packet = {
                "left_state": left_state,
                "right_state": right_state,
                "left_action": left_action,
                "right_action": right_action,
            }
            with self._record_lock:
                self._latest_hand_packet = hand_packet
                self._got_hand_packet = True
            return True

        if not self._warned_bad_hand_packet:
            logger.warning("ArmTeleopCtrl received invalid Inspire hand packet schema.")
            self._warned_bad_hand_packet = True
        return False

    def _toggle_recording(self):
        if self.recorder is None:
            logger.warning("ArmTeleopCtrl recording requested but recorder is not initialized.")
            return

        if not self._record_running:
            if self.recorder.create_episode():
                self._record_running = True
                logger.info("ArmTeleopCtrl recording started.")
        else:
            self._record_running = False
            self.recorder.save_episode()
            logger.info("ArmTeleopCtrl recording stopped.")

    def _capture_record_images(self):
        colors = {}
        depths = {}

        if self.img_client is None:
            return colors, depths

        camera_cfg = self._camera_config
        head_img = None
        left_wrist_img = None
        right_wrist_img = None

        head_cfg = camera_cfg.get("head_camera", {})
        if head_cfg.get("enable_zmq") or head_cfg.get("enable_webrtc"):
            try:
                head_img, _ = self.img_client.get_head_frame()
            except Exception as exc:
                logger.warning("ArmTeleopCtrl failed to capture head frame: %s", exc)

        if camera_cfg.get("left_wrist_camera", {}).get("enable_zmq"):
            try:
                left_wrist_img, _ = self.img_client.get_left_wrist_frame()
            except Exception as exc:
                logger.warning("ArmTeleopCtrl failed to capture left wrist frame: %s", exc)

        if camera_cfg.get("right_wrist_camera", {}).get("enable_zmq"):
            try:
                right_wrist_img, _ = self.img_client.get_right_wrist_frame()
            except Exception as exc:
                logger.warning("ArmTeleopCtrl failed to capture right wrist frame: %s", exc)

        if head_cfg.get("binocular"):
            if head_img is not None:
                width = int(head_cfg.get("image_shape", [0, 0, 0])[1] // 2)
                colors["color_0"] = head_img[:, :width]
                colors["color_1"] = head_img[:, width:]
            if left_wrist_img is not None:
                colors["color_2"] = left_wrist_img
            if right_wrist_img is not None:
                colors["color_3"] = right_wrist_img
        else:
            if head_img is not None:
                colors["color_0"] = head_img
            if left_wrist_img is not None:
                colors["color_1"] = left_wrist_img
            if right_wrist_img is not None:
                colors["color_2"] = right_wrist_img

        return colors, depths

    def _record_step(self):
        if self.recorder is None or not self._record_running:
            return

        with self._record_lock:
            current_arm_q = np.asarray(self._record_snapshot["arm_q"], dtype=np.float32).copy()
            arm_action = np.asarray(self._record_snapshot["arm_action"], dtype=np.float32).copy()
            hand_packet = _copy_hand_packet(self._record_snapshot["hand_packet"])
            got_hand_packet = bool(self._record_snapshot["got_hand_packet"])

        left_ee_state = []
        right_ee_state = []
        left_ee_action = []
        right_ee_action = []

        if self._use_inspire_hand_stream():
            if not got_hand_packet and not self._warned_missing_hand_packet:
                logger.warning("ArmTeleopCtrl recording Inspire hand data, but no hand packet has arrived yet.")
                self._warned_missing_hand_packet = True
            left_ee_state = hand_packet["left_state"]
            right_ee_state = hand_packet["right_state"]
            left_ee_action = hand_packet["left_action"]
            right_ee_action = hand_packet["right_action"]

        colors, depths = self._capture_record_images()
        states = {
            "left_arm": {"qpos": current_arm_q[:7].tolist(), "qvel": [], "torque": []},
            "right_arm": {"qpos": current_arm_q[-7:].tolist(), "qvel": [], "torque": []},
            "left_ee": {"qpos": left_ee_state, "qvel": [], "torque": []},
            "right_ee": {"qpos": right_ee_state, "qvel": [], "torque": []},
            "body": {"qpos": []},
        }
        actions = {
            "left_arm": {"qpos": arm_action[:7].tolist(), "qvel": [], "torque": []},
            "right_arm": {"qpos": arm_action[-7:].tolist(), "qvel": [], "torque": []},
            "left_ee": {"qpos": left_ee_action, "qvel": [], "torque": []},
            "right_ee": {"qpos": right_ee_action, "qvel": [], "torque": []},
            "body": {"qpos": []},
        }
        self.recorder.add_item(colors=colors, depths=depths, states=states, actions=actions)

    def _record_loop(self):
        while not self._stop_event.is_set():
            loop_start = time.time()
            if self._record_running and self.recorder is not None:
                try:
                    queue_obj = getattr(self.recorder, "item_data_queue", None)
                    queue_size = queue_obj.qsize() if queue_obj is not None else 0
                    if queue_size >= self.cfg_ctrl.record_max_queue_size:
                        now = time.time()
                        if (now - self._last_record_backpressure_log_t) > 1.0:
                            logger.warning(
                                "ArmTeleopCtrl recorder backlog=%s, dropping this record tick to protect control timing.",
                                queue_size,
                            )
                            self._last_record_backpressure_log_t = now
                    else:
                        self._record_step()
                except Exception as exc:
                    logger.warning("ArmTeleopCtrl record step failed: %s", exc)
            sleep_time = self._record_dt - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _set_manual_enabled(self, enabled: bool, source: str):
        if enabled:
            if self._manual_enabled.is_set():
                return
            self._manual_enabled.set()
            logger.info("ArmTeleopCtrl enabled via %s", source)
            return

        if not self._manual_enabled.is_set():
            return
        self._manual_enabled.clear()
        logger.info("ArmTeleopCtrl paused via %s", source)

    def _start_manual_listener(self):
        if not self.cfg_ctrl.manual_enable:
            return

        enable_key = self.cfg_ctrl.enable_key
        disable_key = self.cfg_ctrl.disable_key
        record_key = self.cfg_ctrl.record_toggle_key
        logger.info(
            "ArmTeleopCtrl manual gate active. Press '%s' to enable arm teleop, '%s' to pause it, and '%s' to toggle recording.",
            enable_key,
            disable_key,
            record_key,
        )

        def _handle_key(key: str):
            if key == enable_key == disable_key:
                self._set_manual_enabled(not self._manual_enabled.is_set(), f"key '{key}'")
                return
            if key == enable_key:
                self._set_manual_enabled(True, f"key '{key}'")
            elif key == disable_key:
                self._set_manual_enabled(False, f"key '{key}'")
            elif self.cfg_ctrl.record and key == record_key and self._manual_enabled.is_set():
                self._record_toggle_requested = True

        try:
            from sshkeyboard import listen_keyboard, stop_listening
        except Exception as exc:
            logger.warning("ArmTeleopCtrl could not start sshkeyboard listener, falling back to stdin: %s", exc)

            def _stdin_loop():
                while not self._stop_event.is_set():
                    try:
                        raw = input()
                    except EOFError:
                        return
                    key = raw.strip().lower()
                    if key:
                        _handle_key(key)

            self._manual_listener_thread = threading.Thread(
                target=_stdin_loop,
                daemon=True,
                name="arm-teleop-manual-stdin",
            )
            self._manual_listener_thread.start()
            return

        self._manual_listener_stop = stop_listening
        self._manual_listener_thread = threading.Thread(
            target=listen_keyboard,
            kwargs={
                "on_press": _handle_key,
                "until": None,
                "sequential": False,
            },
            daemon=True,
            name="arm-teleop-manual-keyboard",
        )
        self._manual_listener_thread.start()

    def _run_loop(self):
        dt = 1.0 / self.cfg_ctrl.frequency
        while not self._stop_event.is_set():
            start_time = time.time()
            tv_active = False
            zmq_active = False

            try:
                if self.tv_wrapper is not None:
                    tv_active = self._update_from_televuer()
                if self.wrist_zmq_sub is not None:
                    zmq_active = self._update_from_wrist_zmq()
                if self.hand_zmq_sub is not None:
                    self._update_from_hand_zmq()
            except Exception as exc:
                if not self._warned_no_source:
                    logger.warning("ArmTeleopCtrl teleop source update failed: %s", exc)
                    self._warned_no_source = True

            current_arm_q, current_arm_dq = self.env.get_current_joint_state(self.ARM_JOINT_NAMES)
            now = time.time()
            if tv_active or zmq_active:
                self._last_live_input_t = now
                self._warned_waiting_for_input = False
            input_live = (now - self._last_live_input_t) <= self.cfg_ctrl.live_input_timeout

            if self._manual_enabled.is_set() and input_live:
                if (now - self._last_ik_solve_t) >= self._ik_dt:
                    sol_q, sol_tauff = self.arm_ik.solve_ik(
                        self.left_wrist_pose,
                        self.right_wrist_pose,
                        current_lr_arm_motor_q=current_arm_q,
                        current_lr_arm_motor_dq=current_arm_dq,
                    )
                    self._last_sol_q = np.asarray(sol_q, dtype=np.float32).copy()
                    self._last_sol_tauff = np.asarray(sol_tauff, dtype=np.float32).copy()
                    self._last_ik_solve_t = now
                self.env.set_joint_override_targets(
                    self.ARM_JOINT_NAMES,
                    self._last_sol_q,
                    tau=self._last_sol_tauff,
                    ttl_s=self.cfg_ctrl.override_ttl,
                )
            else:
                if self._manual_enabled.is_set() and not input_live and not self._warned_waiting_for_input:
                    logger.info("ArmTeleopCtrl enabled but waiting for live TeleVuer/ZMQ wrist input.")
                    self._warned_waiting_for_input = True
                self._last_sol_q = current_arm_q.copy()
                self._last_sol_tauff = np.zeros_like(current_arm_q)
                self.env.set_joint_override_targets(
                    self.ARM_JOINT_NAMES,
                    current_arm_q,
                    tau=np.zeros_like(current_arm_q),
                    ttl_s=self.cfg_ctrl.override_ttl,
                )

            self._last_ctrl_data = {
                "source": "arm_teleop",
                "active": self._manual_enabled.is_set(),
                "manual_enabled": self._manual_enabled.is_set(),
                "input_live": input_live,
                "televuer_live": tv_active,
                "wrist_zmq_live": zmq_active,
                "recording": self._record_running,
                "timestamp": time.time(),
            }

            with self._record_lock:
                self._record_snapshot["arm_q"] = current_arm_q.copy()
                self._record_snapshot["arm_action"] = self._last_sol_q.copy()
                self._record_snapshot["hand_packet"] = _copy_hand_packet(self._latest_hand_packet)
                self._record_snapshot["got_hand_packet"] = self._got_hand_packet

            if self._record_toggle_requested:
                self._toggle_recording()
                self._record_toggle_requested = False

            sleep_time = dt - (time.time() - start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
