# Cap math-library thread pools for control-loop stability.
import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import argparse
import logging
import time
from pathlib import Path

import robojudo.pipeline
from robojudo.config.config_manager import ConfigManager
from robojudo.pipeline.pipeline_cfgs import RlPipelineCfg
from robojudo.pipeline.rl_pipeline import RlPipeline

logger = logging.getLogger("robojudo")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        default="g1",
        help="Name of the config class to use",
    )
    parser.add_argument("--iface", type=str, help="DDS network interface override")
    parser.add_argument("--record", action="store_true", help="Enable teleop-style arm recording in the merged stack")
    parser.add_argument("--record-frequency", type=float, default=30.0, help="Recording frequency in Hz")
    parser.add_argument("--task-dir", type=str, help="Directory where episodes will be stored")
    parser.add_argument("--task-name", type=str, default="g1_inspire_live", help="Recording task name")
    parser.add_argument("--task-goal", type=str, default="task goal", help="Recording task goal")
    parser.add_argument("--task-desc", type=str, default="task description", help="Recording task description")
    parser.add_argument("--task-steps", type=str, default="step1: do this; step2: do that;", help="Recording task steps")
    parser.add_argument(
        "--ee",
        type=str,
        choices=["dex1", "dex3", "inspire_ftp", "inspire_dfx", "brainco"],
        help="End-effector type for recording metadata",
    )
    parser.add_argument("--img-server-ip", type=str, help="Image server IP for teleimager recording")
    parser.add_argument("--wrist-zmq-host", type=str, help="Wrist ZMQ host override")
    parser.add_argument("--wrist-zmq-port", type=int, help="Wrist ZMQ port override")
    parser.add_argument("--hand-zmq-host", type=str, help="Inspire hand ZMQ host override")
    parser.add_argument("--hand-zmq-port", type=int, help="Inspire hand ZMQ port override")
    args = parser.parse_args()
    return args


def apply_cli_overrides(cfg: RlPipelineCfg, args):
    if args.iface and hasattr(cfg.env, "unitree"):
        cfg.env.unitree.net_if = args.iface

    controlboard_root = Path(__file__).resolve().parents[2]
    record_task_dir = args.task_dir or str(controlboard_root / "teleop_data")

    for ctrl_cfg in getattr(cfg, "ctrl", []) or []:
        if getattr(ctrl_cfg, "ctrl_type", None) != "ArmTeleopCtrl":
            continue

        if args.wrist_zmq_host:
            ctrl_cfg.wrist_zmq_host = args.wrist_zmq_host
        if args.wrist_zmq_port is not None:
            ctrl_cfg.wrist_zmq_port = args.wrist_zmq_port
        if args.hand_zmq_host:
            ctrl_cfg.hand_zmq_host = args.hand_zmq_host
        if args.hand_zmq_port is not None:
            ctrl_cfg.hand_zmq_port = args.hand_zmq_port
        if args.img_server_ip:
            ctrl_cfg.img_server_ip = args.img_server_ip
        if args.ee is not None:
            ctrl_cfg.ee = args.ee

        if args.record:
            ctrl_cfg.record = True
            ctrl_cfg.record_frequency = args.record_frequency
            ctrl_cfg.task_dir = record_task_dir
            ctrl_cfg.task_name = args.task_name
            ctrl_cfg.task_goal = args.task_goal
            ctrl_cfg.task_desc = args.task_desc
            ctrl_cfg.task_steps = args.task_steps

    return cfg


def main():
    args = parse_args()
    logger.info(f"Using config: {args.config}")
    config_manager = ConfigManager(config_name=args.config)

    cfg: RlPipelineCfg = config_manager.get_cfg()
    cfg = apply_cli_overrides(cfg, args)

    pipeline_type = cfg.pipeline_type

    pipeline_class: type[RlPipeline] = getattr(robojudo.pipeline, pipeline_type)
    logger.info(f"Using pipeline: {pipeline_type} -> {pipeline_class}")

    pipeline = pipeline_class(cfg=cfg)

    if not cfg.env.is_sim:
        pipeline.prepare()

    warn_drop_s = max(0.010, 0.5 * pipeline.dt)
    critical_drop_s = 0.2
    hard_drop_s = 1.0
    excessive_drop_count = 0

    while True:
        time_start = time.perf_counter()
        pipeline.step()
        time_end = time.perf_counter()
        time_diff = time_end - time_start

        # keep the pipeline running at the desired frequency
        if not cfg.run_fullspeed:
            time_diff = pipeline.dt - time_diff
            if time_diff > 0:
                time.sleep(time_diff)
            else:
                if not cfg.env.is_sim:
                    if time_diff < -warn_drop_s:
                        logger.warning(f"Warning: frame drop -> {time_diff}")
                    if time_diff < -hard_drop_s:
                        logger.critical("Exiting due to severe frame stall")
                        pipeline.env.shutdown()
                        time.sleep(10)
                        break
                    if time_diff < -critical_drop_s:
                        excessive_drop_count += 1
                        logger.error(
                            "Excessive frame drop count=%s -> %s",
                            excessive_drop_count,
                            time_diff,
                        )
                        if excessive_drop_count >= 3:
                            logger.critical("Exiting due to sustained excessive frame drop")
                            pipeline.env.shutdown()
                            time.sleep(10)
                            break
                    else:
                        excessive_drop_count = 0


if __name__ == "__main__":
    main()
