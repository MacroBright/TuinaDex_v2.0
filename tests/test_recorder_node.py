"""test_recorder_node.py — 采集节点服务生命周期与 bag 落盘 (需 ROS2)."""
import pytest
rclpy = pytest.importorskip("rclpy")

import sys
from pathlib import Path
SRC = Path(__file__).resolve().parents[1] / "src" / "tuinadex_recorder"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from tuinadex_recorder.recorder_node import RecorderNode
from std_srvs.srv import Trigger


def test_start_stop_creates_bag(tmp_path):
    rclpy.init()
    try:
        node = RecorderNode()
        node._bag_root = tmp_path   # 直接注入临时目录 (测试专用)

        resp = Trigger.Response()
        node._on_start(Trigger.Request(), resp)
        assert resp.success is True
        assert (tmp_path / Path(resp.message).name).exists()   # bag 目录已建

        resp2 = Trigger.Response()
        node._on_start(Trigger.Request(), resp2)
        assert resp2.success is False                           # 重复 start 拒绝

        node._on_stop(Trigger.Request(), Trigger.Response())
        resp3 = Trigger.Response()
        node._on_stop(Trigger.Request(), resp3)
        assert resp3.success is False                           # 未在录制
        node.destroy_node()
    finally:
        rclpy.shutdown()
