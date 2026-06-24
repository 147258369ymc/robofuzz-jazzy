import importlib
import os
import sqlite3
import subprocess as sp
import sys
import tempfile
import types
import unittest
from unittest import mock


SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
REPO_ROOT = os.path.dirname(SRC_DIR)
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)


def _install_executor_fakes():
    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy_qos = types.ModuleType("rclpy.qos")
    fake_rclpy_qos.QoSProfile = object
    fake_rclpy_qos.HistoryPolicy = object
    fake_rclpy_qos.DurabilityPolicy = object

    rosidl_runtime_py = types.ModuleType("rosidl_runtime_py")
    rosidl_runtime_py.message_to_ordereddict = lambda msg: {}
    rosidl_runtime_py.message_to_yaml = lambda msg: "{}"

    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.UInt64 = object
    std_msgs_pkg = types.ModuleType("std_msgs")
    std_msgs_pkg.msg = std_msgs_msg

    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Empty = object
    std_srvs_pkg = types.ModuleType("std_srvs")
    std_srvs_pkg.srv = std_srvs_srv

    constants = types.ModuleType("constants")
    ros_utils = types.ModuleType("ros_utils")

    return mock.patch.dict(
        sys.modules,
        {
            "rclpy": fake_rclpy,
            "rclpy.qos": fake_rclpy_qos,
            "rosidl_runtime_py": rosidl_runtime_py,
            "std_msgs": std_msgs_pkg,
            "std_msgs.msg": std_msgs_msg,
            "std_srvs": std_srvs_pkg,
            "std_srvs.srv": std_srvs_srv,
            "constants": constants,
            "ros_utils": ros_utils,
        },
    )


class RuntimeCleanupTests(unittest.TestCase):
    def test_rosbag_parser_closes_sqlite_connection(self):
        fake_rclpy_serialization = types.ModuleType("rclpy.serialization")
        fake_rclpy_serialization.deserialize_message = lambda data, msg_type: data
        fake_rosidl_utilities = types.ModuleType("rosidl_runtime_py.utilities")
        fake_rosidl_utilities.get_message = lambda type_name: object

        with mock.patch.dict(
            sys.modules,
            {
                "rclpy.serialization": fake_rclpy_serialization,
                "rosidl_runtime_py.utilities": fake_rosidl_utilities,
            },
        ):
            sys.modules.pop("rosbag_parser", None)
            rosbag_parser = importlib.import_module("rosbag_parser")

        with tempfile.NamedTemporaryFile(suffix=".db3") as fp:
            conn = sqlite3.connect(fp.name)
            conn.execute(
                "CREATE TABLE topics("
                "id INTEGER, name TEXT, type TEXT, serialization_format TEXT)"
            )
            conn.execute("CREATE TABLE messages(id INTEGER, topic_id INTEGER, timestamp INTEGER, data BLOB)")
            conn.commit()
            conn.close()

            parser = rosbag_parser.RosbagParser(fp.name)

            parser.close()

            self.assertIsNone(parser.cursor)
            self.assertIsNone(parser.conn)

    def test_executor_starts_rosbag_with_devnull_streams(self):
        with _install_executor_fakes():
            sys.modules.pop("executor", None)
            executor = importlib.import_module("executor")

        class _Cfg:
            src_dir = SRC_DIR

        class _Fuzzer:
            config = _Cfg()

        with tempfile.TemporaryDirectory() as td:
            watchlist = os.path.join(td, "watchlist.json")
            with open(watchlist, "w", encoding="utf-8") as fp:
                fp.write('["/joint_states"]')
            _Cfg.watchlist = watchlist

            record = {}

            class _Proc:
                pid = 123

                def poll(self):
                    return None

            def _fake_popen(cmd, **kwargs):
                record["cmd"] = cmd
                record["kwargs"] = kwargs
                return _Proc()

            with mock.patch.object(executor.sp, "Popen", side_effect=_fake_popen):
                with mock.patch.object(executor.time, "sleep", return_value=None):
                    ex = executor.Executor(_Fuzzer())
                    cwd = os.getcwd()
                    try:
                        os.chdir(td)
                        ex.start_rosbag(0)
                    finally:
                        os.chdir(cwd)

        self.assertEqual(sp.DEVNULL, record["kwargs"]["stdout"])
        self.assertEqual(sp.DEVNULL, record["kwargs"]["stderr"])

    def test_run_target_has_explicit_moveit_rviz_disable_switch(self):
        run_target = os.path.join(REPO_ROOT, "run_target.sh")
        with open(run_target, "r", encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("MOVEIT_WITH_RVIZ=1|0", text)
        self.assertIn('MOVEIT_WITH_RVIZ:-1', text)
        self.assertIn("moveit2_panda_headless.launch.py", text)
