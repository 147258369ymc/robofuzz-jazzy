import importlib
import math
import os
import sys
import tempfile
import time
import types
import unittest
from contextlib import redirect_stdout
from collections import deque
from io import StringIO
from unittest import mock


SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)


def import_ulg_parser_with_fake_pyulog():
    pyulog = types.ModuleType("pyulog")
    pyulog.ULog = object
    with mock.patch.dict(sys.modules, {"pyulog": pyulog}):
        sys.modules.pop("ulg_parser", None)
        return importlib.import_module("ulg_parser")


def import_px4_utils_with_fakes():
    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.spin_once = lambda *_args, **_kwargs: None

    class FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    class FakeNode:
        def __init__(self):
            self.publishers = {}
            self.subscriptions = {}

        def create_publisher(self, _msg_type, topic, _qos):
            publisher = FakePublisher()
            self.publishers[topic] = publisher
            return publisher

        def create_subscription(self, _msg_type, topic, callback, _qos):
            self.subscriptions[topic] = callback
            return callback

        def destroy_node(self):
            pass

    fake_rclpy.create_node = lambda _name: FakeNode()

    class VehicleCommand:
        VEHICLE_CMD_DO_SET_MODE = 176
        VEHICLE_CMD_DO_SET_PARAMETER = 180
        VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
        VEHICLE_CMD_NAV_TAKEOFF = 22

        def __init__(self):
            self.timestamp = 0
            self.command = 0
            self.param1 = 0.0
            self.param2 = 0.0
            self.target_system = 0
            self.target_component = 0
            self.source_system = 0
            self.source_component = 0
            self.from_external = False

    class TrajectorySetpoint:
        def __init__(self):
            self.timestamp = 0
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.acceleration = [0.0, 0.0, 0.0]
            self.jerk = [0.0, 0.0, 0.0]
            self.yaw = 0.0
            self.yawspeed = 0.0

    class OffboardControlMode:
        def __init__(self):
            self.timestamp = 0
            self.position = False
            self.velocity = False
            self.acceleration = False
            self.attitude = False
            self.body_rate = False
            self.thrust_and_torque = False
            self.direct_actuator = False

    class VehicleStatus:
        NAVIGATION_STATE_OFFBOARD = 14
        ARMING_STATE_ARMED = 2
        ARMING_STATE_DISARMED = 1

        def __init__(self):
            self.timestamp = 0
            self.nav_state = self.NAVIGATION_STATE_OFFBOARD
            self.arming_state = self.ARMING_STATE_DISARMED
            self.pre_flight_checks_pass = False
            self.gcs_connection_lost = True

    class VehicleControlMode:
        def __init__(self):
            self.timestamp = 0
            self.flag_armed = False
            self.flag_control_offboard_enabled = False

    class VehicleLandDetected:
        def __init__(self):
            self.timestamp = 0
            self.landed = True

    class VehicleLocalPosition:
        def __init__(self):
            self.timestamp = 0
            self.xy_valid = False
            self.z_valid = False
            self.v_xy_valid = False
            self.v_z_valid = False
            self.z = 0.0

    class VehicleCommandAck:
        VEHICLE_CMD_RESULT_ACCEPTED = 0
        VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED = 1
        VEHICLE_CMD_RESULT_DENIED = 2

        def __init__(self):
            self.timestamp = 0
            self.command = 0
            self.result = self.VEHICLE_CMD_RESULT_ACCEPTED

    px4_msgs = types.ModuleType("px4_msgs")
    px4_msgs_msg = types.ModuleType("px4_msgs.msg")
    px4_msgs_msg.VehicleCommand = VehicleCommand
    px4_msgs_msg.TrajectorySetpoint = TrajectorySetpoint
    px4_msgs_msg.OffboardControlMode = OffboardControlMode
    px4_msgs_msg.VehicleStatus = VehicleStatus
    px4_msgs_msg.VehicleControlMode = VehicleControlMode
    px4_msgs_msg.VehicleLandDetected = VehicleLandDetected
    px4_msgs_msg.VehicleLocalPosition = VehicleLocalPosition
    px4_msgs_msg.VehicleCommandAck = VehicleCommandAck
    px4_msgs.msg = px4_msgs_msg

    rosidl_runtime_py = types.ModuleType("rosidl_runtime_py")
    rosidl_runtime_py.message_to_ordereddict = lambda msg: {}
    rosidl_runtime_py.set_message_fields = lambda msg, fields: None

    rosidl_parser = types.ModuleType("rosidl_parser")
    rosidl_definition = types.ModuleType("rosidl_parser.definition")
    rosidl_definition.BasicType = lambda _name: object()
    rosidl_parser.definition = rosidl_definition

    pymavlink = types.ModuleType("pymavlink")
    mavutil = types.ModuleType("pymavlink.mavutil")
    mavutil.mavlink_connection = lambda *_args, **_kwargs: None
    mavutil.mavlink = types.SimpleNamespace(
        MAV_PARAM_TYPE_INT32=6,
        MAV_PARAM_TYPE_REAL32=9,
    )
    pymavlink.mavutil = mavutil

    with mock.patch.dict(
        sys.modules,
        {
            "rclpy": fake_rclpy,
            "px4_msgs": px4_msgs,
            "px4_msgs.msg": px4_msgs_msg,
            "rosidl_runtime_py": rosidl_runtime_py,
            "rosidl_parser": rosidl_parser,
            "rosidl_parser.definition": rosidl_definition,
            "pymavlink": pymavlink,
            "pymavlink.mavutil": mavutil,
        },
    ):
        sys.modules.pop("px4_utils", None)
        return importlib.import_module("px4_utils")


def import_scheduler_with_fakes():
    constants = types.ModuleType("constants")
    mutator = types.ModuleType("mutator")
    mutator.gen_special_floats = lambda: 0.0
    harness = types.ModuleType("harness")
    numpy = types.ModuleType("numpy")
    numpy.random = types.SimpleNamespace(
        randint=lambda *args, **kwargs: 0,
        choice=lambda seq, *args, **kwargs: seq[0],
    )
    ros_utils = types.ModuleType("ros_utils")
    ros_utils.flatten_nested_dict = lambda _msg_type_dict: []
    ros2_fuzzer = types.ModuleType("ros2_fuzzer")
    ros_commons = types.ModuleType("ros2_fuzzer.ros_commons")
    ros_commons.map_ros_types = lambda _msg_type_class: {}
    ros2_fuzzer.ros_commons = ros_commons

    with mock.patch.dict(
        sys.modules,
        {
            "constants": constants,
            "mutator": mutator,
            "harness": harness,
            "numpy": numpy,
            "ros_utils": ros_utils,
            "ros2_fuzzer": ros2_fuzzer,
            "ros2_fuzzer.ros_commons": ros_commons,
        },
    ):
        sys.modules.pop("scheduler", None)
        return importlib.import_module("scheduler")


class Px4JazzyAdaptationTests(unittest.TestCase):
    @staticmethod
    def _msg(**kwargs):
        return types.SimpleNamespace(**kwargs)

    @staticmethod
    def _px4_v117_config():
        return types.SimpleNamespace(
            oracle_mode="px4_v117_jazzy",
            use_ulg=True,
            flight_mode="OFFBOARD",
            exp_pgfuzz=False,
        )

    def _normal_px4_v117_state(self):
        msg = self._msg
        return {
            "/VehicleStatus_PubSubTopic": [
                (1_000_000_000, msg(
                    nav_state=14,
                    arming_state=2,
                    failsafe=False,
                )),
                (1_200_000_000, msg(
                    nav_state=14,
                    arming_state=2,
                    failsafe=False,
                )),
            ],
            "/VehicleControlMode_PubSubTopic": [
                (1_000_000_000, msg(
                    flag_armed=True,
                    flag_control_offboard_enabled=True,
                    flag_control_position_enabled=False,
                    flag_control_velocity_enabled=True,
                    flag_control_attitude_enabled=True,
                )),
                (1_200_000_000, msg(
                    flag_armed=True,
                    flag_control_offboard_enabled=True,
                    flag_control_position_enabled=False,
                    flag_control_velocity_enabled=True,
                    flag_control_attitude_enabled=True,
                )),
            ],
            "/VehicleLocalPosition_PubSubTopic": [
                (1_000_000_000, msg(
                    x=0.0, y=0.0, z=-1.2,
                    vx=0.0, vy=0.0, vz=0.0,
                    heading=0.0,
                )),
                (1_100_000_000, msg(
                    x=0.02, y=0.0, z=-1.25,
                    vx=0.2, vy=0.0, vz=-0.1,
                    heading=0.05,
                )),
                (1_200_000_000, msg(
                    x=0.06, y=0.0, z=-1.3,
                    vx=0.3, vy=0.0, vz=-0.1,
                    heading=0.1,
                )),
            ],
            "/VehicleAttitude_PubSubTopic": [
                (1_000_000_000, msg(q=[1.0, 0.0, 0.0, 0.0])),
                (1_100_000_000, msg(q=[0.999, 0.02, 0.0, 0.0])),
                (1_200_000_000, msg(q=[0.999, 0.02, 0.0, 0.0])),
            ],
            "/VehicleAngularVelocity_PubSubTopic": [
                (1_000_000_000, msg(xyz=[0.0, 0.0, 0.0])),
                (1_100_000_000, msg(xyz=[0.1, 0.0, 0.0])),
                (1_200_000_000, msg(xyz=[0.1, 0.0, 0.0])),
            ],
            "/VehicleOdometry_PubSubTopic": [
                (1_100_000_000, msg(
                    x=0.02, y=0.0, z=-1.25,
                    angular_velocity=[0.1, 0.0, 0.0],
                )),
            ],
            "/OffboardControlMode_PubSubTopic": [
                (1_000_000_000, msg(timestamp=1_000_000)),
                (1_100_000_000, msg(timestamp=1_100_000)),
                (1_200_000_000, msg(timestamp=1_200_000)),
            ],
            "/TrajectorySetpoint_PubSubTopic": [
                (1_050_000_000, msg(velocity=[0.2, 0.0, 0.0])),
                (1_150_000_000, msg(velocity=[0.3, 0.0, -0.1])),
            ],
            "/VehicleRatesSetpoint_PubSubTopic": [
                (1_100_000_000, msg(roll=0.1, pitch=0.0, yaw=0.0)),
            ],
            "/VehicleLandDetected_PubSubTopic": [
                (1_000_000_000, msg(landed=False)),
            ],
            "/ActuatorOutputs_PubSubTopic": [
                (1_000_000_000, msg(output=[1200.0, 1200.0,
                                            1200.0, 1200.0])),
            ],
        }

    def test_find_latest_ulg_discovers_px4_sitl_default_logs(self):
        ulg_parser = import_ulg_parser_with_fake_pyulog()

        with tempfile.TemporaryDirectory() as px4_root:
            log_dir = os.path.join(
                px4_root, "build", "px4_sitl_default", "tmp",
                "rootfs", "log", "2026-06-29"
            )
            os.makedirs(log_dir)
            old_ulg = os.path.join(log_dir, "old.ulg")
            new_ulg = os.path.join(log_dir, "new.ulg")
            open(old_ulg, "w", encoding="utf-8").close()
            open(new_ulg, "w", encoding="utf-8").close()
            now = time.time()
            os.utime(old_ulg, (now - 20, now - 20))
            os.utime(new_ulg, (now, now))

            self.assertEqual(
                new_ulg,
                ulg_parser.find_latest_ulg(
                    log_root=None,
                    px4_root=px4_root,
                    since_time=now - 1,
                ),
            )

    def test_px4_v117_ulg_mapping_includes_flight_quality_topics(self):
        ulg_parser = import_ulg_parser_with_fake_pyulog()

        expected = {
            "vehicle_control_mode": "/VehicleControlMode_PubSubTopic",
            "vehicle_land_detected": "/VehicleLandDetected_PubSubTopic",
            "vehicle_local_position_setpoint":
                "/VehicleLocalPositionSetpoint_PubSubTopic",
            "vehicle_command_ack": "/VehicleCommandAck_PubSubTopic",
            "offboard_control_mode": "/OffboardControlMode_PubSubTopic",
            "vehicle_command": "/VehicleCommand_PubSubTopic",
            "failsafe_flags": "/FailsafeFlags_PubSubTopic",
        }

        for ulg_name, ros_topic in expected.items():
            self.assertEqual(ros_topic, ulg_parser.ULG_TO_ROSTOPIC[ulg_name])

    def test_px4_v117_sitl_overrides_use_mavlink_without_switching_ros_mode(self):
        px4_utils = import_px4_utils_with_fakes()

        class FakeMav:
            def __init__(self):
                self.param_sets = []

            def param_set_send(
                self,
                target_system,
                target_component,
                param_name,
                param_value,
                param_type,
            ):
                self.param_sets.append((
                    target_system,
                    target_component,
                    param_name.decode("ascii"),
                    param_value,
                    param_type,
                ))

        class FakeMaster:
            def __init__(self):
                self.target_system = 1
                self.target_component = 1
                self.mav = FakeMav()

            def wait_heartbeat(self, timeout=30):
                return True

        fake_master = FakeMaster()
        bridge = px4_utils.Px4BridgeNode(
            use_mavlink=False,
            target_profile_name="px4_v117_jazzy",
        )
        bridge._mavlink_param_connection = lambda: fake_master

        bridge.apply_v117_sitl_param_overrides()

        self.assertFalse(bridge.use_mavlink)
        param_values = {
            name: value
            for _sys, _comp, name, value, _param_type
            in fake_master.mav.param_sets
        }
        self.assertEqual(0, param_values["NAV_DLL_ACT"])
        self.assertEqual(4, param_values["COM_DLL_EXCEPT"])
        self.assertEqual(4, param_values["COM_RCL_EXCEPT"])
        self.assertEqual(30, param_values["COM_RC_LOSS_T"])

    def test_px4_v117_setpoint_logging_is_summarized_by_default(self):
        px4_utils = import_px4_utils_with_fakes()
        bridge = px4_utils.Px4BridgeNode(
            use_mavlink=False,
            target_profile_name="px4_v117_jazzy",
        )

        with mock.patch.dict(os.environ, {}, clear=True):
            output = StringIO()
            with redirect_stdout(output):
                for _ in range(5):
                    bridge.send_command(px4_utils.TrajectorySetpoint())

        log = output.getvalue()
        self.assertEqual(3, log.count("[offboard controller] sending:"))
        self.assertIn("setpoint logging summarized", log)

    def test_px4_v117_setpoint_logging_can_be_verbose(self):
        px4_utils = import_px4_utils_with_fakes()
        bridge = px4_utils.Px4BridgeNode(
            use_mavlink=False,
            target_profile_name="px4_v117_jazzy",
        )

        with mock.patch.dict(os.environ, {"PX4_VERBOSE_SETPOINTS": "1"}):
            output = StringIO()
            with redirect_stdout(output):
                for _ in range(5):
                    bridge.send_command(px4_utils.TrajectorySetpoint())

        self.assertEqual(
            5,
            output.getvalue().count("[offboard controller] sending:"),
        )

    def test_px4_v117_flight_quality_summary_detects_not_armed_or_airborne(self):
        px4_utils = import_px4_utils_with_fakes()
        bridge = px4_utils.Px4BridgeNode(
            use_mavlink=False,
            target_profile_name="px4_v117_jazzy",
        )

        status = px4_utils.VehicleStatus()
        status.nav_state = status.NAVIGATION_STATE_OFFBOARD
        status.arming_state = status.ARMING_STATE_DISARMED
        status.pre_flight_checks_pass = False
        status.gcs_connection_lost = True
        bridge.vehicle_status_callback(status)

        control = px4_utils.VehicleControlMode()
        control.flag_control_offboard_enabled = True
        control.flag_armed = False
        bridge.vehicle_control_mode_callback(control)

        landed = px4_utils.VehicleLandDetected()
        landed.landed = True
        bridge.vehicle_land_detected_callback(landed)

        summary = bridge.v117_flight_quality_summary()

        self.assertTrue(summary["offboard"])
        self.assertFalse(summary["armed"])
        self.assertFalse(summary["airborne"])
        self.assertIn("not armed", summary["reason"])
        self.assertIn("landed", summary["reason"])

    def test_px4_v117_airborne_requires_meaningful_altitude(self):
        px4_utils = import_px4_utils_with_fakes()
        bridge = px4_utils.Px4BridgeNode(
            use_mavlink=False,
            target_profile_name="px4_v117_jazzy",
        )

        status = px4_utils.VehicleStatus()
        status.arming_state = status.ARMING_STATE_ARMED
        status.pre_flight_checks_pass = True
        bridge.vehicle_status_callback(status)

        control = px4_utils.VehicleControlMode()
        control.flag_armed = True
        control.flag_control_offboard_enabled = True
        bridge.vehicle_control_mode_callback(control)

        landed = px4_utils.VehicleLandDetected()
        landed.landed = False
        bridge.vehicle_land_detected_callback(landed)

        pos = px4_utils.VehicleLocalPosition()
        pos.xy_valid = True
        pos.z_valid = True
        pos.v_xy_valid = True
        pos.v_z_valid = True
        pos.z = -0.05
        bridge.vehicle_local_position_callback(pos)
        self.assertFalse(bridge.v117_flight_quality_summary()["airborne"])

        pos.z = -1.2
        bridge.vehicle_local_position_callback(pos)
        self.assertTrue(bridge.v117_flight_quality_summary()["airborne"])

    def test_px4_v117_velocity_seed_sequences_cover_arrays(self):
        px4_utils = import_px4_utils_with_fakes()

        seeds = px4_utils.generate_v117_offboard_velocity_seeds(
            px4_utils.TrajectorySetpoint,
            seqlen=6,
        )

        self.assertGreaterEqual(len(seeds), 6)
        self.assertTrue(any(
            any(abs(msg.velocity[0]) > 0.0 for msg in seq)
            for seq in seeds
        ))
        self.assertTrue(any(
            any(abs(msg.velocity[1]) > 0.0 for msg in seq)
            for seq in seeds
        ))
        self.assertTrue(any(
            any(abs(msg.velocity[2]) > 0.0 for msg in seq)
            for seq in seeds
        ))
        self.assertTrue(any(
            any(abs(msg.yawspeed) > 0.0 for msg in seq)
            for seq in seeds
        ))
        for seq in seeds:
            for msg in seq:
                self.assertTrue(all(math.isnan(v) for v in msg.position))

    def test_px4_v117_velocity_seed_sequences_do_not_duplicate_hover_baseline(self):
        px4_utils = import_px4_utils_with_fakes()

        seeds = px4_utils.generate_v117_offboard_velocity_seeds(
            px4_utils.TrajectorySetpoint,
            seqlen=120,
        )

        self.assertTrue(seeds)
        self.assertTrue(all(len(seq) == 120 for seq in seeds))
        for seq in seeds:
            active = sum(
                any(abs(v) > 1e-6 for v in msg.velocity)
                or abs(msg.yaw) > 1e-6
                or abs(msg.yawspeed) > 1e-6
                for msg in seq
            )
            self.assertGreater(
                active,
                0,
                "v1.17-specific seeds should not repeat the generic hover "
                "baseline already inserted by Fuzzer.init_queue",
            )

    def test_px4_v117_ros_mutation_writes_velocity_array(self):
        px4_utils = import_px4_utils_with_fakes()
        with mock.patch.dict(sys.modules, {"px4_utils": px4_utils}):
            scheduler = import_scheduler_with_fakes()

        class FixedRange:
            def sample(self):
                return 1.25

        class FixedProfile:
            def get_range(self, _field_name):
                return FixedRange()

        sched = object.__new__(scheduler.Scheduler)
        sched.fuzzer = types.SimpleNamespace(
            config=types.SimpleNamespace(
                target_profile_name="px4_v117_jazzy",
            )
        )
        sched.msg_list = [
            px4_utils.TrajectorySetpoint()
            for _ in range(4)
        ]
        sched.msg_field_list = [["vx", "float32"]]

        sched._ros_single_block(
            FixedProfile(),
            start_idx=0,
            block_len=4,
        )

        self.assertTrue(all(
            msg.velocity[0] == 1.25 for msg in sched.msg_list
        ))

    def test_px4_v117_mutation_profile_uses_deep_conservative_ranges(self):
        import mutation_profile

        profile = mutation_profile.MutationProfile.px4_v117_offboard_velocity()

        self.assertEqual((-3.0, 3.0), (
            profile.get_range("vx").low,
            profile.get_range("vx").high,
        ))
        self.assertEqual((-3.0, 3.0), (
            profile.get_range("vy").low,
            profile.get_range("vy").high,
        ))
        self.assertEqual((-1.2, 1.2), (
            profile.get_range("vz").low,
            profile.get_range("vz").high,
        ))
        self.assertEqual((-1.8, 1.8), (
            profile.get_range("yawspeed").low,
            profile.get_range("yawspeed").high,
        ))
        self.assertEqual(0.0, profile.get_range("vx").extreme_prob)
        self.assertEqual((8, 40), profile.block_len_range)

    def test_px4_v117_sequence_mutation_stays_in_deep_velocity_envelope(self):
        px4_utils = import_px4_utils_with_fakes()
        with mock.patch.dict(sys.modules, {"px4_utils": px4_utils}):
            scheduler = import_scheduler_with_fakes()

        with tempfile.TemporaryDirectory() as tmpdir:
            sched = object.__new__(scheduler.Scheduler)
            sched.fuzzer = types.SimpleNamespace(
                config=types.SimpleNamespace(
                    target_profile_name="px4_v117_jazzy",
                ),
                queue=deque(),
            )
            sched.msg_type_class = px4_utils.TrajectorySetpoint
            sched.msg_field_list = [
                ["vx", "float32"],
                ["vy", "float32"],
                ["vz", "float32"],
                ["yaw", "float32"],
                ["yawspeed", "float32"],
            ]
            sched.num_msgs = 12
            sched.subscriber_node = "/fmu/in/trajectory_setpoint"
            sched.topic_name = "/fmu/in/trajectory_setpoint"
            sched.cycle_cnt = 0
            sched.round_cnt = 0
            sched.is_new_cycle = True

            config = types.SimpleNamespace(
                meta_dir=tmpdir,
                target_profile_name="px4_v117_jazzy",
            )
            msg_list, _frame = sched.mutate_sequence_ros(config)

        self.assertTrue(msg_list)
        for msg in msg_list:
            self.assertLessEqual(abs(msg.velocity[0]), 3.0)
            self.assertLessEqual(abs(msg.velocity[1]), 3.0)
            self.assertLessEqual(abs(msg.velocity[2]), 1.2)
            self.assertLessEqual(abs(msg.yawspeed), 1.8)

    def test_px4_v117_restart_refuses_legacy_rtps_path(self):
        px4_utils = import_px4_utils_with_fakes()
        bridge = px4_utils.Px4BridgeNode(use_mavlink=True)

        with mock.patch.dict(
            os.environ,
            {"ROBOFUZZ_TARGET_PROFILE": "px4_v117_jazzy"},
            clear=False,
        ):
            with self.assertRaisesRegex(RuntimeError, "px4_v117_jazzy"):
                bridge._restart_px4_stack()

    def test_px4_v117_smoke_oracle_classifies_missing_state(self):
        from oracles import px4

        config = types.SimpleNamespace(oracle_mode="px4_v117_jazzy")
        errs = px4.check(config, [], {}, [])

        self.assertIn(
            "px4_v117_transport_missing: /fmu/out/vehicle_status_v1",
            errs,
        )
        self.assertIn(
            "px4_v117_transport_missing: "
            "/fmu/out/vehicle_local_position_v1",
            errs,
        )
        self.assertIn(
            "px4_v117_transport_missing: /fmu/out/vehicle_attitude",
            errs,
        )

    def test_px4_v117_ulg_oracle_updates_dynamic_feedback_metrics(self):
        from feedback import Feedback, FeedbackType
        from oracles import px4

        def msg(**kwargs):
            return types.SimpleNamespace(**kwargs)

        state_dict = {
            "/VehicleStatus_PubSubTopic": [
                (1_000_000_000, msg(nav_state=14, arming_state=2)),
            ],
            "/VehicleLocalPosition_PubSubTopic": [
                (1_000_000_000, msg(x=0.0, y=0.0, z=-1.0,
                                    vx=0.0, vy=0.0, vz=0.0)),
                (1_200_000_000, msg(x=1.2, y=0.0, z=-2.0,
                                    vx=4.0, vy=3.0, vz=-0.5)),
                (1_420_000_000, msg(x=2.1, y=0.4, z=-3.0,
                                    vx=2.0, vy=1.0, vz=-0.2)),
            ],
            "/VehicleAttitude_PubSubTopic": [
                (1_000_000_000, msg(q=[1.0, 0.0, 0.0, 0.0])),
                (1_200_000_000, msg(q=[0.9659258, 0.2588190, 0.0, 0.0])),
            ],
            "/VehicleAngularVelocity_PubSubTopic": [
                (1_200_000_000, msg(xyz=[0.0, 0.0, 2.0])),
            ],
            "/TrajectorySetpoint_PubSubTopic": [
                (1_200_000_000, msg(velocity=[1.0, 0.0, 0.0])),
            ],
            "/VehicleRatesSetpoint_PubSubTopic": [
                (1_200_000_000, msg(roll=0.0, pitch=0.0, yaw=0.5)),
            ],
        }
        feedback_names = [
            "max_xy_velocity",
            "max_tilt_angle",
            "max_angular_rate",
            "max_altitude",
            "vel_pos_inconsistency",
            "control_loop_gap",
            "px4_v117_velocity_tracking_error",
            "px4_v117_rate_tracking_error",
        ]
        feedback_list = [
            Feedback(name, FeedbackType.INC) for name in feedback_names
        ]
        config = types.SimpleNamespace(
            oracle_mode="px4_v117_jazzy",
            use_ulg=True,
            flight_mode="OFFBOARD",
        )

        errs = px4.check(config, [], state_dict, feedback_list)
        values = {fbk.name: fbk.value for fbk in feedback_list}

        self.assertEqual([], errs)
        self.assertAlmostEqual(5.0, values["max_xy_velocity"])
        self.assertGreater(values["max_tilt_angle"], 25.0)
        self.assertGreater(values["max_angular_rate"], 110.0)
        self.assertEqual(3.0, values["max_altitude"])
        self.assertGreater(values["vel_pos_inconsistency"], 0.0)
        self.assertAlmostEqual(0.22, values["control_loop_gap"])
        self.assertGreater(values["px4_v117_velocity_tracking_error"], 3.0)
        self.assertGreater(values["px4_v117_rate_tracking_error"], 1.0)

    def test_px4_v117_ulg_oracle_classifies_sustained_actuator_saturation(self):
        from feedback import Feedback, FeedbackType
        from oracles import px4

        def msg(**kwargs):
            return types.SimpleNamespace(**kwargs)

        state_dict = {
            "/VehicleStatus_PubSubTopic": [
                (1_000_000_000, msg(nav_state=14, arming_state=2)),
            ],
            "/VehicleLocalPosition_PubSubTopic": [
                (1_000_000_000, msg(x=0.0, y=0.0, z=-1.0,
                                    vx=0.0, vy=0.0, vz=0.0)),
                (1_100_000_000, msg(x=0.1, y=0.0, z=-1.0,
                                    vx=0.0, vy=0.0, vz=0.0)),
            ],
            "/VehicleAttitude_PubSubTopic": [
                (1_000_000_000, msg(q=[1.0, 0.0, 0.0, 0.0])),
            ],
            "/ActuatorOutputs_PubSubTopic": [
                (1_000_000_000, msg(output=[1950.0, 1950.0,
                                            1950.0, 1950.0])),
                (1_160_000_000, msg(output=[1950.0, 1950.0,
                                            1950.0, 1950.0])),
                (1_260_000_000, msg(output=[1950.0, 1950.0,
                                            1950.0, 1950.0])),
            ],
        }
        feedback_list = [Feedback("actuator_saturation", FeedbackType.INC)]
        config = types.SimpleNamespace(
            oracle_mode="px4_v117_jazzy",
            use_ulg=True,
            flight_mode="OFFBOARD",
        )

        errs = px4.check(config, [], state_dict, feedback_list)

        self.assertTrue(any(
            "px4_v117_actuator_saturation" in err for err in errs
        ))
        self.assertGreater(feedback_list[0].value, 0.2)

    def test_px4_v117_ulg_oracle_ignores_disarmed_low_motor_outputs(self):
        from feedback import Feedback, FeedbackType
        from oracles import px4

        def msg(**kwargs):
            return types.SimpleNamespace(**kwargs)

        state_dict = {
            "/VehicleStatus_PubSubTopic": [
                (1_000_000_000, msg(nav_state=14, arming_state=1)),
            ],
            "/VehicleLocalPosition_PubSubTopic": [
                (1_000_000_000, msg(x=0.0, y=0.0, z=0.0,
                                    vx=0.0, vy=0.0, vz=0.0)),
                (1_100_000_000, msg(x=0.0, y=0.0, z=0.0,
                                    vx=0.0, vy=0.0, vz=0.0)),
            ],
            "/VehicleAttitude_PubSubTopic": [
                (1_000_000_000, msg(q=[1.0, 0.0, 0.0, 0.0])),
            ],
            "/ActuatorOutputs_PubSubTopic": [
                (1_000_000_000, msg(output=[1000.0, 1000.0,
                                            1000.0, 1000.0])),
                (2_000_000_000, msg(output=[1000.0, 1000.0,
                                            1000.0, 1000.0])),
                (3_000_000_000, msg(output=[1000.0, 1000.0,
                                            1000.0, 1000.0])),
            ],
        }
        feedback_list = [Feedback("actuator_saturation", FeedbackType.INC)]
        config = types.SimpleNamespace(
            oracle_mode="px4_v117_jazzy",
            use_ulg=True,
            flight_mode="OFFBOARD",
        )

        errs = px4.check(config, [], state_dict, feedback_list)

        self.assertEqual([], errs)
        self.assertEqual(0.0, feedback_list[0].value)

    def test_px4_v117_defaults_match_generated_oracleir(self):
        from oracles import px4

        self.assertEqual(12.0, px4.PX4_DEFAULTS["MPC_XY_VEL_MAX"])
        self.assertEqual(3.0, px4.PX4_DEFAULTS["MPC_ACC_HOR"])
        self.assertEqual(5.0, px4.PX4_DEFAULTS["MPC_ACC_HOR_MAX"])
        self.assertEqual(1.5, px4.PX4_DEFAULTS["MPC_Z_VEL_MAX_DN"])
        self.assertEqual(8.0, px4.PX4_DEFAULTS["MPC_JERK_MAX"])
        self.assertEqual(45.0, px4.PX4_DEFAULTS["MPC_TILTMAX_AIR"])

    def test_px4_v117_normal_offboard_flight_has_no_deep_oracle_errors(self):
        from feedback import Feedback, FeedbackType
        from oracles import px4

        feedback_names = [
            "px4_v117_horizontal_speed_ratio",
            "px4_v117_climb_speed_ratio",
            "px4_v117_descent_speed_ratio",
            "px4_v117_horizontal_accel_ratio",
            "px4_v117_vertical_accel_ratio",
            "px4_v117_jerk_ratio",
            "px4_v117_tilt_ratio",
            "px4_v117_roll_rate_ratio",
            "px4_v117_offboard_heartbeat_gap_ratio",
            "px4_v117_control_mode_cascade_pressure",
            "px4_v117_quality_score",
        ]
        feedback_list = [
            Feedback(name, FeedbackType.INC) for name in feedback_names
        ]

        errs = px4.check(
            self._px4_v117_config(),
            [],
            self._normal_px4_v117_state(),
            feedback_list,
        )
        values = {fbk.name: fbk.value for fbk in feedback_list}

        self.assertEqual([], errs)
        self.assertGreater(values["px4_v117_quality_score"], 0.0)
        self.assertLess(values["px4_v117_horizontal_speed_ratio"], 0.1)
        self.assertEqual(0.0, values["px4_v117_control_mode_cascade_pressure"])

    def test_px4_v117_horizontal_velocity_violation_is_classified(self):
        from oracles import px4

        state_dict = self._normal_px4_v117_state()
        state_dict["/VehicleLocalPosition_PubSubTopic"] = [
            (1_000_000_000, self._msg(x=0.0, y=0.0, z=-2.0,
                                      vx=13.0, vy=0.0, vz=0.0,
                                      heading=0.0)),
            (1_100_000_000, self._msg(x=1.3, y=0.0, z=-2.0,
                                      vx=13.2, vy=0.0, vz=0.0,
                                      heading=0.0)),
        ]

        errs = px4.check(self._px4_v117_config(), [], state_dict, [])

        self.assertTrue(any(
            "px4_v117_velocity_bound_violation" in err for err in errs
        ))

    def test_px4_v117_vertical_velocity_violation_uses_ned_sign(self):
        from oracles import px4

        state_dict = self._normal_px4_v117_state()
        state_dict["/VehicleLocalPosition_PubSubTopic"] = [
            (1_000_000_000, self._msg(x=0.0, y=0.0, z=-2.0,
                                      vx=0.0, vy=0.0, vz=-3.8,
                                      heading=0.0)),
            (1_100_000_000, self._msg(x=0.0, y=0.0, z=-2.4,
                                      vx=0.0, vy=0.0, vz=-3.7,
                                      heading=0.0)),
            (1_200_000_000, self._msg(x=0.0, y=0.0, z=-2.1,
                                      vx=0.0, vy=0.0, vz=2.2,
                                      heading=0.0)),
            (1_300_000_000, self._msg(x=0.0, y=0.0, z=-1.9,
                                      vx=0.0, vy=0.0, vz=2.3,
                                      heading=0.0)),
        ]

        errs = px4.check(self._px4_v117_config(), [], state_dict, [])

        self.assertTrue(any("climb" in err for err in errs))
        self.assertTrue(any("descent" in err for err in errs))

    def test_px4_v117_accel_violation_remains_error_but_jerk_is_feedback(self):
        from oracles import px4

        state_dict = self._normal_px4_v117_state()
        state_dict["/VehicleLocalPosition_PubSubTopic"] = [
            (1_000_000_000, self._msg(x=0.0, y=0.0, z=-2.0,
                                      vx=0.0, vy=0.0, vz=0.0,
                                      heading=0.0)),
            (1_100_000_000, self._msg(x=0.2, y=0.0, z=-2.0,
                                      vx=0.2, vy=0.0, vz=0.0,
                                      heading=0.0)),
            (1_200_000_000, self._msg(x=1.0, y=0.0, z=-2.0,
                                      vx=1.0, vy=0.0, vz=0.0,
                                      heading=0.0)),
            (1_300_000_000, self._msg(x=2.5, y=0.0, z=-2.0,
                                      vx=2.5, vy=0.0, vz=0.0,
                                      heading=0.0)),
            (1_400_000_000, self._msg(x=5.0, y=0.0, z=-2.0,
                                      vx=5.0, vy=0.0, vz=0.0,
                                      heading=0.0)),
            (1_500_000_000, self._msg(x=8.5, y=0.0, z=-2.0,
                                      vx=8.5, vy=0.0, vz=0.0,
                                      heading=0.0)),
        ]

        errs = px4.check(self._px4_v117_config(), [], state_dict, [])

        self.assertTrue(any(
            "px4_v117_accel_bound_violation" in err for err in errs
        ))
        self.assertFalse(any(
            "px4_v117_jerk_bound_violation" in err for err in errs
        ))

    def test_px4_v117_jerk_oracle_ignores_single_sample_velocity_spike(self):
        from feedback import Feedback, FeedbackType
        from oracles import px4

        state_dict = self._normal_px4_v117_state()
        state_dict["/VehicleLocalPosition_PubSubTopic"] = [
            (1_000_000_000, self._msg(x=0.0, y=0.0, z=-2.0,
                                      vx=0.0, vy=0.0, vz=0.0,
                                      heading=0.0)),
            (1_100_000_000, self._msg(x=0.0, y=0.0, z=-2.0,
                                      vx=0.0, vy=0.0, vz=0.0,
                                      heading=0.0)),
            (1_200_000_000, self._msg(x=0.01, y=0.0, z=-2.0,
                                      vx=0.1, vy=0.0, vz=0.0,
                                      heading=0.0)),
            (1_300_000_000, self._msg(x=0.0, y=0.0, z=-2.0,
                                      vx=0.0, vy=0.0, vz=0.0,
                                      heading=0.0)),
            (1_400_000_000, self._msg(x=0.0, y=0.0, z=-2.0,
                                      vx=0.0, vy=0.0, vz=0.0,
                                      heading=0.0)),
        ]
        feedback_list = [
            Feedback("px4_v117_jerk_ratio", FeedbackType.INC),
        ]

        errs = px4.check(
            self._px4_v117_config(), [], state_dict, feedback_list
        )

        self.assertFalse(any(
            "px4_v117_jerk_bound_violation" in err for err in errs
        ))
        self.assertGreater(feedback_list[0].value, 0.0)

    def test_px4_v117_control_mode_cascade_violation(self):
        from oracles import px4

        state_dict = self._normal_px4_v117_state()
        state_dict["/VehicleControlMode_PubSubTopic"] = [
            (1_000_000_000, self._msg(
                flag_armed=True,
                flag_control_offboard_enabled=True,
                flag_control_position_enabled=True,
                flag_control_velocity_enabled=False,
                flag_control_attitude_enabled=True,
            )),
            (1_100_000_000, self._msg(
                flag_armed=True,
                flag_control_offboard_enabled=True,
                flag_control_position_enabled=False,
                flag_control_velocity_enabled=True,
                flag_control_attitude_enabled=False,
            )),
        ]

        errs = px4.check(self._px4_v117_config(), [], state_dict, [])

        self.assertTrue(any(
            "px4_v117_control_mode_cascade_violation" in err
            for err in errs
        ))

    def test_px4_v117_offboard_heartbeat_gap_is_quality_or_state_error(self):
        from oracles import px4

        state_dict = self._normal_px4_v117_state()
        state_dict["/OffboardControlMode_PubSubTopic"] = []

        no_heartbeat_errs = px4.check(
            self._px4_v117_config(), [], state_dict, []
        )
        self.assertFalse(any(
            "px4_v117_offboard_state_contradiction" in err
            for err in no_heartbeat_errs
        ))

        state_dict["/OffboardControlMode_PubSubTopic"] = [
            (1_000_000_000, self._msg(timestamp=1_000_000)),
            (2_200_000_000, self._msg(timestamp=2_200_000)),
        ]
        state_dict["/VehicleStatus_PubSubTopic"] = [
            (1_000_000_000, self._msg(
                nav_state=14, arming_state=2, failsafe=False)),
            (2_200_000_000, self._msg(
                nav_state=14, arming_state=2, failsafe=False)),
        ]

        errs = px4.check(self._px4_v117_config(), [], state_dict, [])

        self.assertTrue(any(
            "px4_v117_offboard_state_contradiction" in err for err in errs
        ))

    def test_px4_v117_failsafe_anomaly(self):
        from oracles import px4

        state_dict = self._normal_px4_v117_state()
        state_dict["/VehicleStatus_PubSubTopic"] = [
            (1_000_000_000, self._msg(
                nav_state=14, arming_state=2, failsafe=True)),
            (1_100_000_000, self._msg(
                nav_state=14, arming_state=2, failsafe=True)),
        ]

        errs = px4.check(self._px4_v117_config(), [], state_dict, [])

        self.assertTrue(any(
            "px4_v117_failsafe_anomaly" in err for err in errs
        ))

    def test_px4_v117_failsafe_with_explanatory_signal_loss_is_not_bug(self):
        from feedback import Feedback, FeedbackType
        from oracles import px4

        state_dict = self._normal_px4_v117_state()
        state_dict["/VehicleStatus_PubSubTopic"] = [
            (1_000_000_000, self._msg(
                timestamp=1_000_000,
                nav_state=14,
                arming_state=2,
                failsafe=False,
            )),
            (1_100_000_000, self._msg(
                timestamp=1_100_000,
                nav_state=5,
                arming_state=2,
                failsafe=True,
            )),
        ]
        state_dict["/FailsafeFlags_PubSubTopic"] = [
            (1_100_000_000, self._msg(
                timestamp=1_100_000,
                offboard_control_signal_lost=True,
                manual_control_signal_lost=True,
                gcs_connection_lost=True,
            )),
        ]
        feedback_list = [
            Feedback("px4_v117_failsafe_pressure", FeedbackType.INC),
        ]

        errs = px4.check(
            self._px4_v117_config(), [], state_dict, feedback_list
        )

        self.assertFalse(any(
            "px4_v117_failsafe_anomaly" in err for err in errs
        ))
        self.assertEqual(1.0, feedback_list[0].value)

    def test_px4_v117_offboard_velocity_mode_selector_violation(self):
        from feedback import Feedback, FeedbackType
        from oracles import px4

        state_dict = self._normal_px4_v117_state()
        state_dict["/OffboardControlMode_PubSubTopic"] = [
            (1_000_000_000, self._msg(
                timestamp=1_000_000,
                position=False,
                velocity=True,
                acceleration=False,
                attitude=False,
                body_rate=False,
                thrust_and_torque=False,
                direct_actuator=False,
            )),
            (1_100_000_000, self._msg(
                timestamp=1_100_000,
                position=True,
                velocity=False,
                acceleration=False,
                attitude=False,
                body_rate=False,
                thrust_and_torque=False,
                direct_actuator=False,
            )),
        ]
        feedback_list = [
            Feedback("px4_v117_offboard_mode_mismatch_score",
                     FeedbackType.INC),
        ]

        errs = px4.check(
            self._px4_v117_config(), [], state_dict, feedback_list
        )

        self.assertTrue(any(
            "px4_v117_offboard_input_contract_violation" in err
            for err in errs
        ))
        self.assertEqual(1.0, feedback_list[0].value)

    def test_px4_v117_velocity_mode_trajectory_setpoint_contract(self):
        from feedback import Feedback, FeedbackType
        from oracles import px4

        state_dict = self._normal_px4_v117_state()
        state_dict["/OffboardControlMode_PubSubTopic"] = [
            (1_000_000_000, self._msg(
                timestamp=1_000_000,
                position=False,
                velocity=True,
                acceleration=False,
                attitude=False,
                body_rate=False,
                thrust_and_torque=False,
                direct_actuator=False,
            )),
            (1_100_000_000, self._msg(
                timestamp=1_100_000,
                position=False,
                velocity=True,
                acceleration=False,
                attitude=False,
                body_rate=False,
                thrust_and_torque=False,
                direct_actuator=False,
            )),
        ]
        state_dict["/TrajectorySetpoint_PubSubTopic"] = [
            (1_000_000_000, self._msg(
                timestamp=1_000_000,
                position=[math.nan, math.nan, math.nan],
                velocity=[0.0, 0.0, 0.0],
                acceleration=[math.nan, math.nan, math.nan],
                yaw=0.0,
                yawspeed=0.0,
            )),
            (1_100_000_000, self._msg(
                timestamp=1_100_000,
                position=[1.0, math.nan, math.nan],
                velocity=[0.2, 0.0, 0.0],
                acceleration=[math.nan, math.nan, math.nan],
                yaw=0.0,
                yawspeed=0.0,
            )),
        ]
        feedback_list = [
            Feedback("px4_v117_setpoint_contract_pressure",
                     FeedbackType.INC),
        ]

        errs = px4.check(
            self._px4_v117_config(), [], state_dict, feedback_list
        )

        self.assertTrue(any(
            "px4_v117_setpoint_contract_violation" in err
            for err in errs
        ))
        self.assertEqual(1.0, feedback_list[0].value)

    def test_px4_v117_velocity_mode_local_setpoint_contract(self):
        from feedback import Feedback, FeedbackType
        from oracles import px4

        state_dict = self._normal_px4_v117_state()
        state_dict["/OffboardControlMode_PubSubTopic"] = [
            (1_000_000_000, self._msg(
                timestamp=1_000_000,
                position=False,
                velocity=True,
                acceleration=False,
                attitude=False,
                body_rate=False,
                thrust_and_torque=False,
                direct_actuator=False,
            )),
            (1_100_000_000, self._msg(
                timestamp=1_100_000,
                position=False,
                velocity=True,
                acceleration=False,
                attitude=False,
                body_rate=False,
                thrust_and_torque=False,
                direct_actuator=False,
            )),
        ]
        state_dict["/VehicleLocalPositionSetpoint_PubSubTopic"] = [
            (1_000_000_000, self._msg(
                timestamp=1_000_000,
                x=math.nan,
                y=math.nan,
                z=math.nan,
                vx=0.0,
                vy=0.0,
                vz=0.0,
                yaw=0.0,
                yawspeed=0.0,
            )),
            (1_100_000_000, self._msg(
                timestamp=1_100_000,
                x=0.5,
                y=math.nan,
                z=math.nan,
                vx=0.2,
                vy=0.0,
                vz=0.0,
                yaw=0.0,
                yawspeed=0.0,
            )),
        ]
        feedback_list = [
            Feedback("px4_v117_local_setpoint_contract_pressure",
                     FeedbackType.INC),
        ]

        errs = px4.check(
            self._px4_v117_config(), [], state_dict, feedback_list
        )

        self.assertTrue(any(
            "px4_v117_setpoint_contract_violation" in err
            for err in errs
        ))
        self.assertEqual(1.0, feedback_list[0].value)

    def test_px4_v117_attitude_setpoint_contract_detects_invalid_output(self):
        from feedback import Feedback, FeedbackType
        from oracles import px4

        state_dict = self._normal_px4_v117_state()
        state_dict["/OffboardControlMode_PubSubTopic"] = [
            (1_000_000_000, self._msg(
                timestamp=1_000_000,
                position=False,
                velocity=True,
                acceleration=False,
                attitude=False,
                body_rate=False,
                thrust_and_torque=False,
                direct_actuator=False,
            )),
            (1_100_000_000, self._msg(
                timestamp=1_100_000,
                position=False,
                velocity=True,
                acceleration=False,
                attitude=False,
                body_rate=False,
                thrust_and_torque=False,
                direct_actuator=False,
            )),
        ]
        state_dict["/VehicleAttitudeSetpoint_PubSubTopic"] = [
            (1_000_000_000, self._msg(
                timestamp=1_000_000,
                q_d=[1.0, 0.0, 0.0, 0.0],
                thrust_body=[0.0, 0.0, -0.5],
                yaw_sp_move_rate=0.0,
            )),
            (1_100_000_000, self._msg(
                timestamp=1_100_000,
                q_d=[2.0, 0.0, 0.0, 0.0],
                thrust_body=[0.0, 0.0, -1.2],
                yaw_sp_move_rate=0.0,
            )),
        ]
        feedback_list = [
            Feedback("px4_v117_attitude_setpoint_pressure",
                     FeedbackType.INC),
        ]

        errs = px4.check(
            self._px4_v117_config(), [], state_dict, feedback_list
        )

        self.assertTrue(any(
            "px4_v117_attitude_setpoint_invalid" in err for err in errs
        ))
        self.assertGreater(feedback_list[0].value, 0.0)

    def test_px4_v117_feedback_registration_and_mapping(self):
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        mutation_profile_path = os.path.join(SRC_DIR, "mutation_profile.py")
        with open(fuzzer_path, "r", encoding="utf-8") as fp:
            fuzzer_text = fp.read()
        with open(mutation_profile_path, "r", encoding="utf-8") as fp:
            profile_text = fp.read()

        feedback_names = [
            "px4_v117_horizontal_speed_ratio",
            "px4_v117_climb_speed_ratio",
            "px4_v117_descent_speed_ratio",
            "px4_v117_horizontal_accel_ratio",
            "px4_v117_vertical_accel_ratio",
            "px4_v117_jerk_ratio",
            "px4_v117_tilt_ratio",
            "px4_v117_roll_rate_ratio",
            "px4_v117_pitch_rate_ratio",
            "px4_v117_yaw_rate_ratio",
            "px4_v117_offboard_heartbeat_gap_ratio",
            "px4_v117_control_mode_cascade_pressure",
            "px4_v117_failsafe_pressure",
            "px4_v117_command_ack_anomaly_score",
            "px4_v117_actuator_saturation_ratio",
            "px4_v117_quality_score",
            "px4_v117_offboard_mode_mismatch_score",
            "px4_v117_setpoint_contract_pressure",
            "px4_v117_local_setpoint_contract_pressure",
            "px4_v117_attitude_setpoint_pressure",
        ]
        for name in feedback_names:
            self.assertIn(name, fuzzer_text)
            self.assertIn(name, profile_text)

    def test_px4_v117_error_signature_dedup(self):
        import ast

        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, "r", encoding="utf-8") as fp:
            module = ast.parse(fp.read(), filename=fuzzer_path)
        func = next(
            node for node in module.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "px4_error_signature"
        )
        compiled = compile(
            ast.Module(body=[func], type_ignores=[]),
            fuzzer_path,
            "exec",
        )
        namespace = {}
        exec(compiled, namespace)
        px4_error_signature = namespace["px4_error_signature"]

        sigs = px4_error_signature([
            "px4_v117_velocity_bound_violation: horizontal ratio=1.2",
            "px4_v117_accel_bound_violation: vertical ratio=2.0",
            "px4_v117_attitude_rate_violation: angular_rate axis=yaw ratio=5.7",
            "px4_v117_attitude_rate_violation: tilt ratio=1.6",
            "px4_v117_failsafe_anomaly: spurious failsafe",
            "px4_v117_offboard_input_contract_violation: expected velocity mode",
            "px4_v117_setpoint_contract_violation: velocity mode position finite",
            "px4_v117_attitude_setpoint_invalid: thrust_body outside [-1, 1]",
        ])

        self.assertIn(("velocity_bound", "horizontal", 2), sigs)
        self.assertIn(("accel_bound", "vertical", 4), sigs)
        self.assertIn(("attitude_rate", "yaw", 11), sigs)
        self.assertIn(("attitude_rate", "tilt", 3), sigs)
        self.assertIn(("failsafe", "", 0), sigs)
        self.assertIn(("offboard_input_contract", "", 0), sigs)
        self.assertIn(("setpoint_contract", "", 0), sigs)
        self.assertIn(("attitude_setpoint", "", 0), sigs)

    def test_px4_v117_error_summary_writer_records_feedback_and_signatures(self):
        import ast
        import numpy as np
        from feedback import Feedback, FeedbackType

        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, "r", encoding="utf-8") as fp:
            module = ast.parse(fp.read(), filename=fuzzer_path)
        funcs = [
            node for node in module.body
            if isinstance(node, ast.FunctionDef)
            and node.name in {
                "px4_error_signature",
                "write_px4_error_summary",
            }
        ]
        compiled = compile(
            ast.Module(body=funcs, type_ignores=[]),
            fuzzer_path,
            "exec",
        )
        namespace = {}
        exec(compiled, namespace)

        feedback_list = [
            Feedback("px4_v117_vertical_accel_ratio", FeedbackType.INC),
            Feedback("px4_v117_jerk_ratio", FeedbackType.INC),
            Feedback("px4_v117_quality_score", FeedbackType.INC),
        ]
        feedback_list[0].update_value(np.float32(1.82))
        feedback_list[1].update_value(np.float64(5.7))
        feedback_list[2].update_value(1.0)

        with tempfile.TemporaryDirectory() as tmpdir:
            config = types.SimpleNamespace(error_dir=tmpdir)
            namespace["write_px4_error_summary"](
                config,
                "frame-1",
                ["px4_v117_accel_bound_violation: vertical ratio=1.82"],
                feedback_list,
                "/tmp/example.ulg",
                {("accel_bound", "vertical", 3)},
            )
            summary_path = os.path.join(tmpdir, "error-frame-1.summary.json")
            self.assertTrue(os.path.exists(summary_path))
            with open(summary_path, "r", encoding="utf-8") as fp:
                summary = fp.read()

        self.assertIn("px4_v117_accel_bound_violation", summary)
        self.assertIn("px4_v117_vertical_accel_ratio", summary)
        self.assertIn("accel_bound", summary)
        self.assertIn("/tmp/example.ulg", summary)


if __name__ == "__main__":
    unittest.main()
