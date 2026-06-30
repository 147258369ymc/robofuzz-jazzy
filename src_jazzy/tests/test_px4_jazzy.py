import importlib
import math
import os
import sys
import tempfile
import time
import types
import unittest
from collections import deque
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

    def test_px4_v117_mutation_profile_uses_smoke_safe_ranges(self):
        import mutation_profile

        profile = mutation_profile.MutationProfile.px4_v117_offboard_velocity()

        self.assertEqual((-2.0, 2.0), (
            profile.get_range("vx").low,
            profile.get_range("vx").high,
        ))
        self.assertEqual((-2.0, 2.0), (
            profile.get_range("vy").low,
            profile.get_range("vy").high,
        ))
        self.assertEqual((-1.0, 1.0), (
            profile.get_range("vz").low,
            profile.get_range("vz").high,
        ))
        self.assertEqual((-1.2, 1.2), (
            profile.get_range("yawspeed").low,
            profile.get_range("yawspeed").high,
        ))
        self.assertEqual(0.0, profile.get_range("vx").extreme_prob)

    def test_px4_v117_sequence_mutation_stays_in_safe_velocity_envelope(self):
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
            self.assertLessEqual(abs(msg.velocity[0]), 2.0)
            self.assertLessEqual(abs(msg.velocity[1]), 2.0)
            self.assertLessEqual(abs(msg.velocity[2]), 1.0)
            self.assertLessEqual(abs(msg.yawspeed), 1.2)

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


if __name__ == "__main__":
    unittest.main()
