#!/usr/bin/env python3
"""collect_teleop_data.py — TuinaDex v2.0 统一多模态数据采集终端入口.

支持:
1. 仿真/真机一行切换 (--env sim|real)
2. 自适应 16D 手部 / 22D 臂手协同 (--mode hand_only|arm_hand)
3. 多摄像头 RGB 图像流同步 (--cameras head,wrist)
4. 直通 LeRobotDataset v3.0 视频分块与 Parquet 导出
5. 自动化 Mock 示教与交互式遥操示教
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
import numpy as np

# 确保源码路径可导入
ROOT = Path(__file__).resolve().parents[1]
for p in ("src/tuinadex_core", "src/tuinadex_teleop", "src/tuinadex_sim", "dataset"):
    full_p = str(ROOT / p)
    if full_p not in sys.path:
        sys.path.insert(0, full_p)

from tuinadex_teleop.envs.config import RobotConfig
from tuinadex_teleop.envs import make_tuina_env
from tuinadex_teleop.collection.collector import TuinaCollector, CollectionState
from tuinadex_dataset.lerobot_exporter import LeRobotDatasetWriter

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("collect_teleop")


def parse_args():
    parser = argparse.ArgumentParser(description="TuinaDex Multi-Modal Data Collector")
    parser.add_argument("--env", choices=["sim", "real"], default="sim", help="Environment type (sim or real)")
    parser.add_argument("--mode", choices=["hand_only", "arm_hand"], default="hand_only", help="Robot joint mode")
    parser.add_argument(
        "--cameras",
        default="overhead,wrist,side",
        help="Comma-separated camera names (e.g. overhead,wrist,side)",
    )
    parser.add_argument("--fps", type=float, default=50.0, help="Sampling & control frequency in Hz")
    parser.add_argument("--repo-id", default="data/tuina_demo", help="Target LeRobot dataset directory")
    parser.add_argument("--task", default="tuina_massage", help="Task prompt description")
    parser.add_argument("--mock-teleop", action="store_true", help="Use synthetic mock teleoperation for testing")
    parser.add_argument("--episodes", type=int, default=1, help="Number of episodes to collect in mock mode")
    parser.add_argument("--frames-per-episode", type=int, default=50, help="Frames per episode in mock mode")
    parser.add_argument("--standalone", action="store_true", help="Run MuJoCo sim in standalone in-process mode")
    parser.add_argument("--hub", action="store_true", help="Launch the web UI hub instead of terminal collection")
    parser.add_argument("--host", default="127.0.0.1", help="Hub bind address (default loopback only)")
    parser.add_argument("--port", type=int, default=8080, help="Hub port (0 picks a free port)")
    parser.add_argument("--headless", action="store_true", help="Do not open a MuJoCo viewer")
    parser.add_argument("--ui-fps", type=float, default=25.0, help="SSE telemetry push rate")
    return parser.parse_args()


def _mock_action(config: RobotConfig, k: int) -> np.ndarray:
    """Generate the deterministic smooth mock trajectory."""
    if config.mode == "hand_only":
        return (
            0.3
            + 0.2
            * np.sin(
                0.1 * k + np.linspace(0, np.pi, 16)
            ).astype(np.float32)
        )
    arm_act = (
        0.2 * np.sin(0.05 * k + np.arange(6)).astype(np.float32)
    )
    hand_act = (
        0.3
        + 0.2
        * np.sin(
            0.1 * k + np.linspace(0, np.pi, 16)
        ).astype(np.float32)
    )
    return np.concatenate([arm_act, hand_act]).astype(np.float32)


def run_mock_collection(collector: TuinaCollector, config: RobotConfig, episodes: int, frames: int, fps: float):
    """自动化生成测试示教轨迹并保存."""
    logger.info(f"Starting automated mock collection: {episodes} episode(s), {frames} frames/ep at {fps}Hz...")
    dt = 1.0 / fps

    for ep in range(episodes):
        collector.start_recording()
        logger.info(f">>> Recording Episode {ep + 1}/{episodes}...")
        for k in range(frames):
            action = _mock_action(config, k)

            collector.step_once(action)
            # 在独立模式下无需高延迟 sleep，轻微休眠保证稳定
            time.sleep(0.001)

        collector.stop_recording()
        collector.review_decision(save=True)

    logger.info(f"Automated collection completed. Total saved: {collector.total_episodes_saved}.")


def run_hub_session(args, config: RobotConfig):
    """Run the web Hub with dedicated rendering and async dataset saving."""
    import os
    import signal
    import threading

    os.environ.setdefault("MUJOCO_GL", "egl")

    from tuinadex_teleop.adapters.mujoco_sim_adapter import MujocoSimAdapter
    from tuinadex_teleop.cameras.sim_camera_rig import CameraProducerThread
    from tuinadex_teleop.collection.data_channel import (
        RealDataChannel,
        SimDataChannel,
    )
    from tuinadex_teleop.collection.data_hub_controller import (
        DataHubController,
    )
    from tuinadex_teleop.core.types import (
        ClutchState,
        TeleopArmCommand,
        TeleopCommand,
        TeleopHandCommand,
    )
    from tuinadex_teleop.hub.server import CollectionHubServer

    if config.mode != "arm_hand":
        raise SystemExit(
            "--hub requires --mode arm_hand (22-DOF contract, invariant 26)"
        )

    writer = LeRobotDatasetWriter(
        repo_id=args.repo_id,
        config=config,
        fps=args.fps,
        use_videos=True,
    )
    collector = TuinaCollector(env=None)
    from tuinadex_teleop.adapters import DEFAULT_READY_ARM_Q
    init_q = np.zeros(config.action_dim, dtype=np.float32)
    if config.action_dim >= 6:
        init_q[:6] = DEFAULT_READY_ARM_Q.copy()
    commanded = {
        "q": init_q
    }

    adapter = None
    producer = None
    env = None
    lock = threading.Lock()

    if args.env == "sim":
        adapter = MujocoSimAdapter(headless=args.headless)
        if not adapter.connect():
            raise RuntimeError("MujocoSimAdapter connection failed")
        producer = CameraProducerThread(
            adapter.model,
            adapter.data,
            lock,
            target_fps=args.fps,
            jpeg_quality=90,
        )
        producer.start()
        if not producer.wait_ready(30.0):
            raise RuntimeError(
                "Camera producer failed to start: "
                f"{producer.stats()['last_error']}"
            )
        channel = SimDataChannel(
            adapter,
            producer,
            action_supplier=lambda: commanded["q"],
        )
    else:
        env = make_tuina_env(env_type="real", config=config)
        channel = RealDataChannel(
            env, action_supplier=lambda: commanded["q"]
        )

    sub_node = None
    home_pub = None
    bridge_proc = None
    stop_rx = threading.Event()
    _cmd_rx_count = 0

    # 1. Fast UDP command listener (seamless across Python 3.10 & 3.14)
    import socket, struct, subprocess

    def _udp_rx_worker():
        nonlocal _cmd_rx_count
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("127.0.0.1", 9876))
            sock.settimeout(0.5)
            while not stop_rx.is_set():
                try:
                    data, _ = sock.recvfrom(128)
                    if len(data) >= 92 and data[:4] == b"TD22":
                        q_vals = struct.unpack("<4s22f", data)[1:]
                        with lock:
                            commanded["q"] = np.array(q_vals, dtype=np.float32)
                            _cmd_rx_count += 1
                            if _cmd_rx_count == 1:
                                print(
                                    "[INFO] >>> First teleop command received via UDP! Simulation is now tracking teleop hand commands in real-time. <<<",
                                    flush=True,
                                )
                except socket.timeout:
                    continue
                except Exception:
                    break
        finally:
            sock.close()

    udp_th = threading.Thread(target=_udp_rx_worker, daemon=True)
    udp_th.start()

    # 2. Native ROS 2 subscription or auto-spawned bridge
    try:
        import rclpy
        from sensor_msgs.msg import JointState
        if not rclpy.ok():
            rclpy.init()
        sub_node = rclpy.create_node("collect_teleop_cmd_sub")

        def _arm_cmd_cb(msg: JointState):
            nonlocal _cmd_rx_count
            with lock:
                if len(msg.position) >= 6:
                    commanded["q"][:6] = np.array(msg.position[:6], dtype=np.float32)
                    _cmd_rx_count += 1
                    if _cmd_rx_count == 1:
                        print("[INFO] >>> First /arm/joint_command received! Simulation is now tracking teleop hand commands in real-time. <<<", flush=True)

        def _hand_cmd_cb(msg: JointState):
            with lock:
                if len(msg.position) >= 16:
                    commanded["q"][6:22] = np.array(msg.position[:16], dtype=np.float32)

        sub_node.create_subscription(JointState, "/arm/joint_command", _arm_cmd_cb, 10)
        sub_node.create_subscription(JointState, "/hand/joint_command", _hand_cmd_cb, 10)
        from std_msgs.msg import Empty
        home_pub = sub_node.create_publisher(Empty, "/teleop/home", 10)
        spin_thread = threading.Thread(target=rclpy.spin, args=(sub_node,), daemon=True)
        spin_thread.start()
        logger.info("Connected to ROS 2 command topics (/arm/joint_command, /hand/joint_command, /teleop/home)")
    except Exception as exc:
        logger.info(f"Direct rclpy unavailable ({exc}); auto-launching background ROS 2 bridge via system python...")
        try:
            bridge_script = str(ROOT / "scripts" / "ros2_teleop_bridge.py")
            bridge_proc = subprocess.Popen(
                ["/usr/bin/python3", bridge_script],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            logger.info("Background ROS 2 teleop bridge started successfully.")
        except Exception as b_exc:
            logger.warning(f"Could not spawn ros2_teleop_bridge: {b_exc}")

    def _do_home():
        with lock:
            if hasattr(config, "action_dim") and config.action_dim >= 22:
                from tuinadex_teleop.adapters import DEFAULT_READY_ARM_Q
                commanded["q"][:6] = DEFAULT_READY_ARM_Q.copy()
                commanded["q"][6:22] = 0.0
            if adapter is not None:
                from tuinadex_teleop.adapters import DEFAULT_READY_ARM_Q
                now = time.monotonic()
                cmd = TeleopCommand(
                    timestamp=now,
                    arm_command=TeleopArmCommand(
                        timestamp=now,
                        target_pose=np.eye(4),
                        target_q=DEFAULT_READY_ARM_Q.copy(),
                    ),
                    hand_command=TeleopHandCommand(
                        timestamp=now,
                        target_q=np.zeros(16, dtype=np.float32),
                    ),
                    clutch_state=ClutchState.DISENGAGED,
                )
                adapter.send_command(cmd)
        # Send UDP HOME trigger to teleop nodes
        try:
            h_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            h_sock.sendto(b"HOME", ("127.0.0.1", 9877))
            h_sock.close()
        except Exception:
            pass
        if sub_node is not None and home_pub is not None:
            try:
                from std_msgs.msg import Empty
                home_pub.publish(Empty())
            except Exception as e:
                logger.debug(f"Failed to publish /teleop/home: {e}")
        logger.info("[HOME] Robot returned to initial ready posture")

    controller = DataHubController(
        channel,
        collector,
        control_fps=args.fps,
        ui_fps=args.ui_fps,
    )
    controller.on_save = lambda episode: writer.save_episode(
        episode, task=args.task
    )
    controller.on_home = _do_home

    server = CollectionHubServer(
        controller, host=args.host, port=args.port
    )
    port = server.start()
    print(f"HUB_READY port={port}", flush=True)

    period = 1.0 / args.fps
    substeps = max(1, int(round(period / 0.002)))
    ui_every = max(1, int(round(args.fps / args.ui_fps)))
    tick = 0
    next_t = time.monotonic()

    def _handle_termination(_signum, _frame):
        raise KeyboardInterrupt

    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGTERM, _handle_termination)
    try:
        while True:
            if args.env == "sim":
                if args.mock_teleop:
                    with lock:
                        commanded["q"] = _mock_action(config, tick)
                tick += 1
                with lock:
                    q = commanded["q"].copy()
                now = time.monotonic()
                command = TeleopCommand(
                    timestamp=now,
                    arm_command=TeleopArmCommand(
                        timestamp=now,
                        target_pose=np.eye(4),
                        target_q=q[:6].copy(),
                    ),
                    hand_command=TeleopHandCommand(
                        timestamp=now, target_q=q[6:].copy()
                    ),
                    clutch_state=ClutchState.ENGAGED,
                )
                with lock:
                    adapter.send_command(command)
                    adapter.step(substeps)
            else:
                env.spin_once()

            controller.tick()
            server.stream.publish_frames(
                channel.get_camera_packets()
            )
            if tick % ui_every == 0:
                server.stream.publish_telemetry(controller.status())

            next_t += period
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()
    except KeyboardInterrupt:
        logger.info("Hub session interrupted by user.")
    finally:
        stop_rx.set()
        if bridge_proc is not None:
            try:
                bridge_proc.terminate()
                bridge_proc.wait(timeout=1.0)
            except Exception:
                pass
        if sub_node is not None:
            sub_node.destroy_node()
        server.stop()
        controller.shutdown()
        writer.close()
        if producer is not None:
            producer.stop()
        if adapter is not None:
            adapter.disconnect()
        if env is not None:
            env.close()
        signal.signal(signal.SIGTERM, previous_sigterm)


def main():
    args = parse_args()
    cam_list = [c.strip() for c in args.cameras.split(",") if c.strip()]
    config = RobotConfig(
        mode=args.mode,
        camera_names=cam_list,
        fps=args.fps,
    )

    logger.info("============================================================")
    logger.info(f" TuinaDex v2.0 Data Collection Session")
    logger.info(f" - Environment: {args.env.upper()} (standalone={args.standalone})")
    logger.info(f" - Mode:        {args.mode} ({config.action_dim}D)")
    logger.info(f" - Cameras:     {config.camera_names}")
    logger.info(f" - Target FPS:  {config.fps} Hz")
    logger.info(f" - Dataset Dir: {args.repo_id}")
    logger.info("============================================================")

    if args.hub:
        run_hub_session(args, config)
        return

    # 1. 实例化统一环境
    env = make_tuina_env(
        env_type=args.env,
        config=config,
        standalone=args.standalone,
    )

    # 2. 实例化 LeRobot v3.0 导出器
    writer = LeRobotDatasetWriter(
        repo_id=args.repo_id,
        config=config,
        fps=args.fps,
        use_videos=True,
    )

    # 3. 实例化采集编排器
    collector = TuinaCollector(
        env=env,
        on_save=lambda buf: writer.save_episode(buf, task=args.task),
    )

    try:
        if args.mock_teleop:
            run_mock_collection(
                collector=collector,
                config=config,
                episodes=args.episodes,
                frames=args.frames_per_episode,
                fps=args.fps,
            )
        else:
            logger.info("Interactive teleop mode. (Use --mock-teleop for automated self-test)")
            # 交互模式骨架
            collector.start_recording()
            try:
                while True:
                    action = np.zeros(config.action_dim, dtype=np.float32)
                    collector.step_once(action)
                    time.sleep(1.0 / config.fps)
            except KeyboardInterrupt:
                logger.info("Interrupted by user.")
                collector.stop_recording()
                collector.review_decision(save=True)
    finally:
        env.close()
        logger.info("Collection session closed.")


if __name__ == "__main__":
    main()
