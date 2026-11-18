import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock


def test_ground_truth_callback_preserves_exact_timestamp(monkeypatch):
    # These stubs allow importing the callback without a ROS installation.
    node_module = ModuleType("rclpy.node")
    node_module.Node = object
    messages = ModuleType("nav_msgs.msg")
    messages.Odometry = object
    for name, module in {
        "rclpy": ModuleType("rclpy"),
        "rclpy.node": node_module,
        "nav_msgs": ModuleType("nav_msgs"),
        "nav_msgs.msg": messages,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location("record_gt", Path(__file__).with_name("record_gt.py"))
    recorder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(recorder)

    stamp = NS(sec=1_700_000_000, nanosec=1)
    pose = NS(position=NS(x=2, y=3, z=4), orientation=NS(w=1, x=0, y=0, z=0))
    msg = NS(header=NS(stamp=stamp), pose=NS(pose=pose))
    writer = Mock()
    recorder.GroundTruthRecorder.listener_callback(NS(writer=writer), msg)
    writer.writerow.assert_called_once_with([1_700_000_000_000_000_001, 2, 3, 4, 1, 0, 0, 0])
