import importlib
import json
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

    def test_executor_kill_rosbag_is_quiet_when_not_started(self):
        with _install_executor_fakes():
            sys.modules.pop("executor", None)
            executor = importlib.import_module("executor")

        class _Cfg:
            src_dir = SRC_DIR

        class _Fuzzer:
            config = _Cfg()

        ex = executor.Executor(_Fuzzer())
        with mock.patch("builtins.print") as print_mock:
            ex.kill_rosbag()

        self.assertEqual([], print_mock.call_args_list)

    def test_executor_stamps_tb4_twiststamped_before_publish(self):
        with _install_executor_fakes():
            sys.modules.pop("executor", None)
            executor = importlib.import_module("executor")

        class _Stamp:
            sec = 0
            nanosec = 0

        class _Header:
            stamp = _Stamp()
            frame_id = ""

        class _Msg:
            header = _Header()

        class _Cfg:
            tb4_sitl = True

        class _Fuzzer:
            config = _Cfg()

        msg = _Msg()
        ex = executor.Executor(_Fuzzer())

        ex.prepare_msg_for_publish(msg)

        self.assertGreater(msg.header.stamp.sec + msg.header.stamp.nanosec, 0)

    def test_executor_writes_tb4_published_command_trace(self):
        with _install_executor_fakes():
            sys.modules.pop("executor", None)
            executor = importlib.import_module("executor")

        class _Linear:
            x = 0.12
            y = 0.0
            z = 0.0

        class _Angular:
            x = 0.0
            y = 0.0
            z = -0.4

        class _Twist:
            linear = _Linear()
            angular = _Angular()

        class _Stamp:
            sec = 123
            nanosec = 456

        class _Header:
            stamp = _Stamp()
            frame_id = ""

        class _Msg:
            header = _Header()
            twist = _Twist()

        class _Cfg:
            tb4_sitl = True
            target_profile_name = "turtlebot4_jazzy"
            meta_dir = None

        class _Fuzzer:
            config = _Cfg()

        with tempfile.TemporaryDirectory() as td:
            _Cfg.meta_dir = td
            ex = executor.Executor(_Fuzzer())
            ex.topic_name = "/cmd_vel"
            ex.msg_typestr = "geometry_msgs/msg/TwistStamped"

            ex.record_published_command(_Msg(), "frame-a", 10.5, 3, True)

            trace_path = os.path.join(td, "published_commands-frame-a.jsonl")
            with open(trace_path, encoding="utf-8") as fp:
                rows = [json.loads(line) for line in fp]

        self.assertEqual(1, len(rows))
        self.assertEqual("/cmd_vel", rows[0]["topic"])
        self.assertEqual(3, rows[0]["sequence_index"])
        self.assertTrue(rows[0]["publish_success"])
        self.assertAlmostEqual(0.12, rows[0]["linear_x"])
        self.assertAlmostEqual(-0.4, rows[0]["angular_z"])

    def test_run_target_has_explicit_moveit_rviz_disable_switch(self):
        run_target = os.path.join(REPO_ROOT, "run_target.sh")
        with open(run_target, "r", encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("MOVEIT_WITH_RVIZ=1|0", text)
        self.assertIn('MOVEIT_WITH_RVIZ:-1', text)
        self.assertIn("moveit2_panda_headless.launch.py", text)

    def test_tb4_docs_require_docker_init_for_repeated_gui_runs(self):
        doc_path = os.path.join(REPO_ROOT, "docs", "TB4_GUI_USAGE.md")
        with open(doc_path, "r", encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("--init", text)
        self.assertIn("zombie", text.lower())

    def test_empty_time_window_log_is_not_reported_as_watch_failure(self):
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, "r", encoding="utf-8") as fp:
            text = fp.read()

        self.assertNotIn('print("[-] watch failed")', text)
        self.assertIn("time-window parser returned no", text)
        self.assertIn("using full rosbag fallback", text)

    def test_profile_target_cleanup_does_not_depend_on_running_flag(self):
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, "r", encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn(
            "if not self.running and self.config.target_profile is None:",
            text,
        )
        self.assertIn("self.ros_pgrp = None", text)

    def test_kill_target_keeps_fuzzer_node_alive_between_profile_rounds(self):
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, "r", encoding="utf-8") as fp:
            text = fp.read()

        kill_target_body = text.split(
            "    def kill_target(self):", 1
        )[1].split("    def kill_monitor(self):", 1)[0]
        destroy_fuzzer_node_body = text.split(
            "    def destroy_fuzzer_node(self):", 1
        )[1].split("    def destroy(self):", 1)[0]
        destroy_body = text.split(
            "    def destroy(self):", 1
        )[1].split("def inspect_target", 1)[0]

        self.assertNotIn("destroy_node()", kill_target_body)
        self.assertNotIn("destroy_fuzzer_node()", kill_target_body)
        self.assertIn("destroy_node()", destroy_fuzzer_node_body)
        self.assertIn("self.destroy_fuzzer_node()", destroy_body)

    def test_rclpy_shutdown_is_idempotent_on_interrupt_cleanup(self):
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, "r", encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("def safe_rclpy_shutdown():", text)
        self.assertNotIn("                rclpy.shutdown()", text)
        self.assertIn("safe_rclpy_shutdown()", text)
