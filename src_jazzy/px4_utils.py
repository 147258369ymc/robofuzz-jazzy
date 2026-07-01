import os
import sys
import time
import json
import pickle
import math

import rclpy
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, OffboardControlMode
try:
    from px4_msgs.msg import (
        VehicleStatus,
        VehicleControlMode,
        VehicleLandDetected,
        VehicleLocalPosition,
        VehicleCommandAck,
    )
except ImportError:
    VehicleStatus = None
    VehicleControlMode = None
    VehicleLandDetected = None
    VehicleLocalPosition = None
    VehicleCommandAck = None
try:
    from px4_msgs.msg import Timesync
except ImportError:
    Timesync = None
try:
    from px4_msgs.msg import ManualControlSetpoint
except ImportError:
    ManualControlSetpoint = None
from rosidl_runtime_py import message_to_ordereddict, set_message_fields
import rosidl_parser.definition

# force MAVLink 2.0
os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil

PX4_V117_OFFBOARD_EXCEPTION_MASK = 1 << 2
PX4_V117_SITL_PARAM_OVERRIDES = (
    ("NAV_DLL_ACT", 0, "INT32"),
    ("COM_DLL_EXCEPT", PX4_V117_OFFBOARD_EXCEPTION_MASK, "INT32"),
    ("COM_RCL_EXCEPT", PX4_V117_OFFBOARD_EXCEPTION_MASK, "INT32"),
    ("COM_RC_LOSS_T", 30.0, "FLOAT"),
)


# TODO: add all parameters and their metadata.
# Jazzy/PX4 v1.17 ROS smoke mode must not require the old v1.12 params.pkl.
param_dict = None
# {name: type, min, max, default, unit}

# param_dict = {
    # "MIS_TAKEOFF_ALT": [0, 2.5, 80],
    # "COM_POS_FS_EPH": [None, 5, None],
    # "EKF2_ABL_LIM": [0.0, 0.0, None],
    # "MC_PITCHRATE_FF": [0.0, 0.0, None],
# }


class Parameter:
    """Class defining a PX4 parameter message"""
    __slots__ = [
        "param_name", "param_type", "min", "max", "_value",
    ]

    _fields_and_field_types = {
        'value': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, param_name=None, value=None):
        self.param_name = param_name
        self.value = value

    def __repr__(self):
        return f"PX4 Param {self.param_name} ({self.param_type}): {self.value} [{self.min}:{self.max}]"

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._value = value


class Px4BridgeNode:
    def __init__(
        self,
        name="px4_bridge",
        use_mavlink=True,
        debug=False,
        target_profile_name=None,
    ):
        self.use_mavlink = use_mavlink
        self.debug = debug
        self.ts = int(time.time() * 1_000_000)
        self.target_profile_name = (
            target_profile_name
            or os.environ.get("ROBOFUZZ_TARGET_PROFILE", "")
        )
        self._v117_vehicle_status = None
        self._v117_vehicle_control_mode = None
        self._v117_vehicle_land_detected = None
        self._v117_vehicle_local_position = None
        self._v117_last_command_ack = None
        self._v117_param_master = None
        self._offboard_send_count = 0

        if not self.use_mavlink:
            self.node_handle = rclpy.create_node(name)

            if Timesync is not None:
                self.sub_timesync = self.node_handle.create_subscription(
                    Timesync,
                    "/fmu/out/timesync",
                    self.timesync_callback,
                    10,
                )

            self.pub_offboard_control_mode = self.node_handle.create_publisher(
                OffboardControlMode, "/fmu/in/offboard_control_mode", 10
            )
            self.pub_vehiclecommand = self.node_handle.create_publisher(
                VehicleCommand, "/fmu/in/vehicle_command", 10
            )
            self.pub_trajectory = self.node_handle.create_publisher(
                TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
            )
            self._create_v117_state_subscriptions()

    def _is_v117_profile(self):
        return self.target_profile_name == "px4_v117_jazzy"

    def _create_subscription_if_available(self, msg_type, topic, callback):
        if msg_type is None:
            return None
        create_subscription = getattr(
            getattr(self, "node_handle", None),
            "create_subscription",
            None,
        )
        if create_subscription is None:
            return None
        try:
            return create_subscription(
                msg_type,
                topic,
                callback,
                _px4_v117_state_qos(),
            )
        except Exception as exc:
            print(f"[px4 v1.17] could not subscribe to {topic}: {exc}")
            return None

    def _create_v117_state_subscriptions(self):
        if not self._is_v117_profile():
            return
        self._create_subscription_if_available(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self.vehicle_status_callback,
        )
        self._create_subscription_if_available(
            VehicleControlMode,
            "/fmu/out/vehicle_control_mode",
            self.vehicle_control_mode_callback,
        )
        self._create_subscription_if_available(
            VehicleLandDetected,
            "/fmu/out/vehicle_land_detected",
            self.vehicle_land_detected_callback,
        )
        self._create_subscription_if_available(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.vehicle_local_position_callback,
        )
        self._create_subscription_if_available(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack",
            self.vehicle_command_ack_callback,
        )

    def vehicle_status_callback(self, msg):
        self._v117_vehicle_status = msg

    def vehicle_control_mode_callback(self, msg):
        self._v117_vehicle_control_mode = msg

    def vehicle_land_detected_callback(self, msg):
        self._v117_vehicle_land_detected = msg

    def vehicle_local_position_callback(self, msg):
        self._v117_vehicle_local_position = msg

    def vehicle_command_ack_callback(self, msg):
        self._v117_last_command_ack = msg

    def init_mavlink(self):
        self.use_mavlink = True
        # Close old connection to release the UDP port
        if hasattr(self, 'master') and self.master:
            try:
                self.master.close()
            except Exception:
                pass
        self.master = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
        self.master.wait_heartbeat(timeout=30)
        if self.master.target_system == 0:
            raise RuntimeError("MAVLink heartbeat timeout")

    def _mavlink_param_connection(self):
        """Create a MAVLink side-channel for parameter setup only.

        PX4 v1.17 Jazzy fuzzing keeps ROS 2 as the command path. This
        connection is deliberately separate from self.master so parameter
        setup does not flip send_command() into MAVLink/manual-control mode.
        """
        if self._v117_param_master is not None:
            return self._v117_param_master

        url = os.environ.get("PX4_MAVLINK_PARAM_URL", "udpin:127.0.0.1:14550")
        master = mavutil.mavlink_connection(url)
        if master is None:
            raise RuntimeError(f"MAVLink parameter connection failed: {url}")
        master.wait_heartbeat(timeout=15)
        if getattr(master, "target_system", 0) == 0:
            raise RuntimeError("MAVLink parameter heartbeat timeout")
        self._v117_param_master = master
        return master

    @staticmethod
    def _mav_param_type(param_type):
        if param_type == "INT32":
            return mavutil.mavlink.MAV_PARAM_TYPE_INT32
        if param_type == "FLOAT":
            return mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        return param_type

    @staticmethod
    def _send_mavlink_param(master, param_name, param_value, param_type):
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            param_name.encode(),
            param_value,
            Px4BridgeNode._mav_param_type(param_type),
        )

    def apply_v117_sitl_param_overrides(self):
        """Prepare PX4 v1.17 x500 SITL for ROS 2 Offboard fuzzing.

        The x500 airframe enables datalink-loss arming checks by default,
        which rejects arming when no GCS is attached. RoboFuzz runs a
        companion-computer Offboard campaign, so we explicitly disable the
        GCS arming requirement and exempt Offboard from data-link/manual
        control loss failsafes. The actual fuzz inputs still go through ROS 2.
        """
        master = self._mavlink_param_connection()
        print("[px4 v1.17] applying SITL Offboard arming parameter overrides")
        for param_name, value, param_type in PX4_V117_SITL_PARAM_OVERRIDES:
            print(f"[*] (MAVLink param) set {param_name} := {value}")
            self._send_mavlink_param(master, param_name, value, param_type)
            time.sleep(0.05)

    def timesync_callback(self, msg):
        self.ts = msg.timestamp

    def get_timestamp(self):
        if Timesync is None or not getattr(self, "ts", None):
            self.ts = int(time.time() * 1_000_000)
        return self.ts

    def publish_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.timestamp = self.get_timestamp()
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        # print(msg)
        self.pub_vehiclecommand.publish(msg)

    def publish_offboard_control_mode(self, use_velocity=False):
        msg = OffboardControlMode()
        msg.timestamp = self.get_timestamp()
        if use_velocity:
            # Velocity control: direct velocity-to-thrust mapping
            _set_if_present(msg, "position", False)
            _set_if_present(msg, "velocity", True)
        else:
            # Position control: position controller tracks velocity setpoints
            # with overshoot potential — better for bug finding
            _set_if_present(msg, "position", True)
            _set_if_present(msg, "velocity", False)
        _set_if_present(msg, "acceleration", False)
        _set_if_present(msg, "attitude", False)
        _set_if_present(msg, "body_rate", False)
        _set_if_present(msg, "thrust_and_torque", False)
        _set_if_present(msg, "direct_actuator", False)
        # print(msg)
        self.pub_offboard_control_mode.publish(msg)

    def publish_arm_command(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.timestamp = self.get_timestamp()
        msg.param1 = 1.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        # print(msg)
        self.pub_vehiclecommand.publish(msg)

    def publish_trajectory(self, msg):
        msg.timestamp = self.get_timestamp()
        self.pub_trajectory.publish(msg)

    def mav_get_param_default(self, param_name):
        params = _load_param_dict()
        return params[param_name]["default"]

    def mav_set_param_msg(self, msg):
        # wrapper for pub_function
        if msg.param_type == "INT32":
            mav_param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
        elif msg.param_type == "FLOAT":
            mav_param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

        self.mav_set_param(msg.param_name, msg.value, mav_param_type)

    def mav_set_param(self, param_name, param_value, param_type):
        print(f"[*] (MAVLink) set param {param_name} := {param_value}")
        self.current_param_name = param_name
        self.current_param_type = param_type

        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            param_name.encode(), # requires 'b'
            param_value,
            param_type,
        )

        # ack_msg = self.master.recv_match(type='PARAM_VALUE', blocking=True)
        # ack_msg = ack_msg.to_dict()
        # print("param set ack:")
        # print(ack_msg)

        if self.debug:
            # Verify parameter change
            self.master.mav.param_request_list_send(
                self.master.target_system,
                self.master.target_component
            )

            while True:
                time.sleep(0.01)
                try:
                    message = self.master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()

                    if message["param_id"] == param_name:
                        print('name: %s\tvalue: %d' % (message['param_id'],
                                                       message['param_value']))
                        break

                except Exception as error:
                    print(error)
                    sys.exit(0)

    def mav_revert_param(self):
        param_name = self.current_param_name
        param_type = self.current_param_type
        value = self.mav_get_param_default(param_name)

        self.mav_set_param(param_name, value, param_type)

    def _v117_spin_once(self):
        rclpy.spin_once(self.node_handle, timeout_sec=0.0)

    def _v117_local_position_ready(self):
        pos = self._v117_vehicle_local_position
        if pos is None:
            return False
        return bool(
            getattr(pos, "xy_valid", False)
            and getattr(pos, "z_valid", False)
            and getattr(pos, "v_xy_valid", False)
            and getattr(pos, "v_z_valid", False)
        )

    def _v117_is_offboard(self):
        status = self._v117_vehicle_status
        control = self._v117_vehicle_control_mode
        nav_offboard = False
        if status is not None:
            nav_offboard_value = getattr(
                VehicleStatus,
                "NAVIGATION_STATE_OFFBOARD",
                14,
            )
            nav_offboard = getattr(status, "nav_state", None) == nav_offboard_value
        control_offboard = bool(
            control is not None
            and getattr(control, "flag_control_offboard_enabled", False)
        )
        return nav_offboard or control_offboard

    def _v117_is_armed(self):
        status = self._v117_vehicle_status
        control = self._v117_vehicle_control_mode
        status_armed = False
        if status is not None:
            armed_value = getattr(VehicleStatus, "ARMING_STATE_ARMED", 2)
            status_armed = getattr(status, "arming_state", None) == armed_value
        control_armed = bool(
            control is not None and getattr(control, "flag_armed", False)
        )
        return status_armed or control_armed

    def _v117_is_airborne(self):
        pos = self._v117_vehicle_local_position
        z = getattr(pos, "z", 0.0) if pos is not None else 0.0
        landed = self._v117_vehicle_land_detected
        try:
            altitude_ready = float(z) < -1.0
        except (TypeError, ValueError):
            return False
        if landed is not None and getattr(landed, "landed", True):
            return False
        return self._v117_is_armed() and altitude_ready

    def _v117_last_ack_summary(self):
        ack = self._v117_last_command_ack
        if ack is None:
            return "no command ack observed"
        return (
            f"last ack command={getattr(ack, 'command', '?')} "
            f"result={getattr(ack, 'result', '?')}"
        )

    def v117_flight_quality_summary(self):
        status = self._v117_vehicle_status
        reasons = []
        ready = self._v117_local_position_ready()
        offboard = self._v117_is_offboard()
        armed = self._v117_is_armed()
        airborne = self._v117_is_airborne()

        if not ready:
            reasons.append("local position not ready")
        if not offboard:
            reasons.append("not in offboard")
        if not armed:
            reasons.append("not armed")
        if self._v117_vehicle_land_detected is not None:
            if getattr(self._v117_vehicle_land_detected, "landed", True):
                reasons.append("landed")
            elif not airborne:
                reasons.append("below takeoff altitude")
        elif not airborne:
            reasons.append("airborne state unavailable")
        if status is not None:
            if getattr(status, "gcs_connection_lost", False):
                reasons.append("gcs lost")
            if getattr(status, "pre_flight_checks_pass", True) is False:
                reasons.append("preflight checks not passing")
        reasons.append(self._v117_last_ack_summary())

        return {
            "local_position_ready": ready,
            "offboard": offboard,
            "armed": armed,
            "airborne": airborne,
            "reason": "; ".join(reasons),
        }

    def _v117_stream_until(
        self,
        predicate,
        timeout,
        setpoint,
        use_velocity=False,
        period=0.1,
    ):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._v117_spin_once()
            self.publish_offboard_control_mode(use_velocity=use_velocity)
            self.publish_trajectory(setpoint)
            self._v117_spin_once()
            if predicate():
                return True
            time.sleep(period)
        return predicate()

    def prepare_v117_offboard_flight(self, mode_str="OFFBOARD"):
        if self.use_mavlink:
            raise RuntimeError(
                "px4_v117_jazzy Offboard quality chain requires ROS 2 "
                "publishing mode"
            )
        if mode_str != "OFFBOARD":
            print(
                f"[px4 v1.17] requested mode {mode_str}; "
                "using OFFBOARD for v1.17 ROS 2 fuzzing"
            )

        self.apply_v117_sitl_param_overrides()

        takeoff_setpoint = TrajectorySetpoint()
        _set_position(takeoff_setpoint, 0.0, 0.0, -5.0)
        _set_velocity(takeoff_setpoint, 0.0, 0.0, 0.0)
        _set_acceleration(takeoff_setpoint, float("nan"), float("nan"), float("nan"))
        if hasattr(takeoff_setpoint, "yaw"):
            takeoff_setpoint.yaw = 0.0
        if hasattr(takeoff_setpoint, "yawspeed"):
            takeoff_setpoint.yawspeed = 0.0

        print("[px4 v1.17] waiting for local position estimate")
        if not self._v117_stream_until(
            self._v117_local_position_ready,
            timeout=10.0,
            setpoint=takeoff_setpoint,
            use_velocity=False,
        ):
            summary = self.v117_flight_quality_summary()
            raise RuntimeError(
                "PX4 v1.17 local position never became ready: "
                f"{summary['reason']}"
            )

        print("[px4 v1.17] engaging OFFBOARD")
        for _ in range(5):
            self._v117_spin_once()
            self.publish_offboard_control_mode(use_velocity=False)
            self.publish_trajectory(takeoff_setpoint)
            self.publish_offboard_mode()
            time.sleep(0.1)

        if not self._v117_stream_until(
            self._v117_is_offboard,
            timeout=5.0,
            setpoint=takeoff_setpoint,
            use_velocity=False,
        ):
            summary = self.v117_flight_quality_summary()
            raise RuntimeError(
                "PX4 v1.17 did not enter OFFBOARD: "
                f"{summary['reason']}"
            )

        print("[px4 v1.17] arming")
        for attempt in range(1, 6):
            self._v117_spin_once()
            self.publish_arm_command()
            if self._v117_stream_until(
                self._v117_is_armed,
                timeout=1.5,
                setpoint=takeoff_setpoint,
                use_velocity=False,
            ):
                break
            print(
                "[px4 v1.17] arm attempt "
                f"{attempt} did not arm ({self._v117_last_ack_summary()})"
            )

        if not self._v117_is_armed():
            summary = self.v117_flight_quality_summary()
            raise RuntimeError(
                "PX4 v1.17 arming failed: "
                f"{summary['reason']}"
            )

        print("[px4 v1.17] waiting for takeoff")
        if not self._v117_stream_until(
            self._v117_is_airborne,
            timeout=12.0,
            setpoint=takeoff_setpoint,
            use_velocity=False,
        ):
            summary = self.v117_flight_quality_summary()
            raise RuntimeError(
                "PX4 v1.17 did not become airborne: "
                f"{summary['reason']}"
            )

        hover_setpoint = get_init_trajectory_msg()
        self._v117_stream_until(
            lambda: True,
            timeout=0.5,
            setpoint=hover_setpoint,
            use_velocity=True,
        )
        summary = self.v117_flight_quality_summary()
        print(
            "[px4 v1.17] flight ready: "
            f"offboard={summary['offboard']} "
            f"armed={summary['armed']} "
            f"airborne={summary['airborne']}"
        )

    def prepare_flight(self, mode_str="MANUAL"):

        if not self.use_mavlink and self._is_v117_profile():
            self.prepare_v117_offboard_flight(mode_str=mode_str)
            return

        if self.use_mavlink:
            # Set COM_RC_LOSS_T = 30 to avoid engaging failsafe
            print("[*] (MAVLink) requesting COM_RC_LOSS_T override")
            self.mav_set_param(
                "COM_RC_LOSS_T",
                30,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

            # Send dummy manual control command to keep the RC connected
            msg = get_init_manual_control_msg()
            self.send_command(msg)

            # mode change won't work without this
            time.sleep(1)

            # Set mode
            self.mav_set_flight_mode(mode_str=mode_str)

            # arm (with retry on failure)
            try:
                self.send_command_arm()
            except RuntimeError as e:
                print(f"[!] {e}, retrying after PX4 restart...")
                self._restart_px4_stack()
                self.init_mavlink()
                self.mav_set_param(
                    "COM_RC_LOSS_T", 30,
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                msg = get_init_manual_control_msg()
                self.send_command(msg)
                time.sleep(1)
                self.mav_set_flight_mode(mode_str=mode_str)
                self.send_command_arm()  # second failure will propagate

        else:
            # The vehicle must be already be receiving a stream of target
            # setpoints (>2Hz) before this mode can be engaged.
            # Use POSITION mode for takeoff.
            dummy = TrajectorySetpoint()
            _set_position(dummy, 0.0, 0.0, -8.0)  # NED: 8m above ground
            _set_velocity(dummy, 0.0, 0.0, 0.0)
            for i in range(50):
                rclpy.spin_once(self.node_handle, timeout_sec=0.0)
                self.publish_offboard_control_mode(use_velocity=False)
                self.publish_trajectory(dummy)
                time.sleep(0.1)

            rclpy.spin_once(self.node_handle, timeout_sec=0.0)
            self.publish_offboard_mode()
            time.sleep(0.1)

            # The vehicle must be armed before this mode can be engaged.
            rclpy.spin_once(self.node_handle, timeout_sec=0.0)
            self.publish_arm_command()
            time.sleep(0.1)

            # Wait for takeoff: keep sending position setpoint until airborne
            # (similar to put_in_air() in MAVLink mode)
            for i in range(80):
                rclpy.spin_once(self.node_handle, timeout_sec=0.0)
                self.publish_offboard_control_mode(use_velocity=False)
                self.publish_trajectory(dummy)
                time.sleep(0.1)

    def send_command_arm(self, timeout=15):
        """Arm the vehicle with a timeout. Raises RuntimeError if arm fails."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

        t_start = time.time()
        while time.time() - t_start < timeout:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                time.sleep(1)
                return
        raise RuntimeError(f"Arm failed: timeout after {timeout}s")

    def _restart_px4_stack(self):
        """Kill PX4 and restart (keep Gazebo alive for reuse)."""
        profile_name = (
            getattr(self, "target_profile_name", "")
            or os.environ.get("ROBOFUZZ_TARGET_PROFILE", "")
        )
        if profile_name == "px4_v117_jazzy":
            raise RuntimeError(
                "px4_v117_jazzy must be restarted through run_target.sh "
                "and its process group; legacy px4_sitl_rtps restart is "
                "disabled for this profile"
            )

        import subprocess as sp
        print("[!] Restarting PX4...")
        sp.run("pkill -9 px4", shell=True, timeout=5,
               stdout=sp.DEVNULL, stderr=sp.DEVNULL)
        try:
            os.remove("/tmp/px4_lock-0")
        except FileNotFoundError:
            pass
        time.sleep(3)
        import harness
        proj_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        harness.run_px4_stack_sh(proj_root)
        time.sleep(12)
        print("[!] PX4 restarted")

    def send_command_takeoff(self, alt=10.0):
        print(f"[+] (MAVLink) send takeoff command")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            alt
        )
        time.sleep(1)

    def send_command(self, msg):
        if self.use_mavlink:
            # print("[mavlink] sending:", msg)
            self.master.mav.manual_control_send(
                self.master.target_system,
                int(msg.x * 1000),
                int(msg.y * 1000),
                int(msg.z * 1000),
                int(msg.r * 1000),
                0
            )

        else:
            rclpy.spin_once(self.node_handle, timeout_sec=0.0)
            # Fuzz phase uses velocity mode so vx/vy/vz are interpreted as
            # velocity setpoints. Position fields must be NaN to prevent
            # PX4 from tracking them as position targets.
            self.publish_offboard_control_mode(use_velocity=True)
            _set_position(msg, float("nan"), float("nan"), float("nan"))
            self.publish_trajectory(msg)
            self._offboard_send_count += 1
            verbose_setpoints = (
                os.environ.get("PX4_VERBOSE_SETPOINTS", "0") == "1"
            )
            if verbose_setpoints or self._offboard_send_count <= 3:
                print("[offboard controller] sending:", msg)
            elif self._offboard_send_count == 4:
                print(
                    "[offboard controller] setpoint logging summarized; "
                    "set PX4_VERBOSE_SETPOINTS=1 for every command"
                )
            elif self._offboard_send_count % 40 == 0:
                print(
                    "[offboard controller] sent "
                    f"{self._offboard_send_count} velocity setpoints"
                )

    def put_in_air(self):
        # Climb with z=900 (strong throttle) for 50 iterations (5s).
        # Reaches ~5m in POSCTL, giving safe buffer for fuzz exploration.
        for i in range(50):
            self.master.mav.manual_control_send(
                self.master.target_system,
                0,
                0,
                900,
                0,
                0
            )
            time.sleep(0.1)

    def mav_set_flight_mode(self, mode_str="MANUAL"):
        # Set manual mode
        (mode, custom_mode, custom_sub_mode) = self.master.mode_mapping()[mode_str]

        self.master.set_mode(mode, custom_mode, custom_sub_mode)

        # cnt = 0
        # while True:
            # cnt += 1
            # if cnt == 100:
                # print(f"[-] (MAVLink) timeout while waiting for set_mode ack")
                # sys.exit(-1)

            # ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            # ack_msg = ack_msg.to_dict()

            # if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                # continue

            # if ack_msg["result"] == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                # print(f"[+] (MAVLink) mode set: {mode_str}")
                # break
            # else:
                # print(f"[-] (MAVLink) failed to set mode {mode_str}")
                # print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
                # sys.exit(-1)
        time.sleep(1)

    def mav_get_status_dict(self):
        try:
            status_dict = self.master.recv_match().to_dict()
        except:
            status_dict = None

        return status_dict

    def takeoff(self):
        print("[offboard controller] take off")
        msg_takeoff = TrajectorySetpoint()
        msg_takeoff.z = -5.0
        for i in range(100):
            self.send_command(msg_takeoff)

    def land(self):
        print("[offboard controller] land")
        msg_land = TrajectorySetpoint()
        msg_land.z = 0.0
        for i in range(100):
            self.send_command(msg_land)

    def shutdown(self):
        self.node_handle.destroy_node()


def get_init_trajectory_msg():
    msg = TrajectorySetpoint()
    _set_position(msg, float("nan"), float("nan"), float("nan"))
    _set_velocity(msg, 0.0, 0.0, 0.0)
    _set_acceleration(msg, float("nan"), float("nan"), float("nan"))
    if hasattr(msg, "yaw"):
        msg.yaw = 0.0
    if hasattr(msg, "yawspeed"):
        msg.yawspeed = 0.0

    return msg


def set_v117_offboard_velocity(
    msg,
    vx,
    vy,
    vz,
    yaw=0.0,
    yawspeed=0.0,
):
    """Prepare a PX4 v1.17 TrajectorySetpoint for velocity Offboard fuzzing."""
    _set_position(msg, float("nan"), float("nan"), float("nan"))
    _set_velocity(msg, vx, vy, vz)
    _set_acceleration(msg, float("nan"), float("nan"), float("nan"))
    if hasattr(msg, "jerk"):
        _set_vector_field(msg, "jerk", (
            float("nan"),
            float("nan"),
            float("nan"),
        ))
    if hasattr(msg, "yaw"):
        msg.yaw = yaw
    if hasattr(msg, "yawspeed"):
        msg.yawspeed = yawspeed
    return msg


def set_v117_velocity_component(msg, field_name, value):
    velocity_index = {"vx": 0, "vy": 1, "vz": 2}
    if field_name in velocity_index:
        values = list(getattr(msg, "velocity", [0.0, 0.0, 0.0]))
        values[velocity_index[field_name]] = value
        _set_velocity(msg, values[0], values[1], values[2])
        _set_position(msg, float("nan"), float("nan"), float("nan"))
        return True
    if field_name in ("yaw", "yawspeed") and hasattr(msg, field_name):
        setattr(msg, field_name, value)
        return True
    return False


def generate_v117_offboard_velocity_seeds(msg_class=TrajectorySetpoint, seqlen=8):
    """Conservative PX4 v1.17 x500 Offboard velocity seed sequences."""
    seeds = []

    def make_msg(vx=0.0, vy=0.0, vz=0.0, yaw=0.0, yawspeed=0.0):
        return set_v117_offboard_velocity(
            msg_class(),
            vx,
            vy,
            vz,
            yaw=yaw,
            yawspeed=yawspeed,
        )

    def append_sequence(fn):
        seq = []
        for idx in range(seqlen):
            phase = idx / max(seqlen - 1, 1)
            seq.append(fn(idx, phase))
        seeds.append(seq)

    append_sequence(lambda _idx, _phase: make_msg(vx=1.5))
    append_sequence(lambda _idx, _phase: make_msg(vy=1.5))
    append_sequence(lambda _idx, _phase: make_msg(vz=-0.5))
    append_sequence(lambda _idx, _phase: make_msg(vx=1.0, vy=1.0))
    append_sequence(lambda _idx, _phase: make_msg(vx=1.0, yawspeed=0.8))
    append_sequence(
        lambda idx, _phase: make_msg(
            vx=1.2 if idx < seqlen // 2 else -1.2
        )
    )
    append_sequence(
        lambda idx, _phase: make_msg(
            vy=1.2 if idx < seqlen // 2 else -1.2
        )
    )
    append_sequence(
        lambda idx, _phase: make_msg(
            vz=-0.45 if idx < seqlen // 2 else 0.45
        )
    )
    append_sequence(
        lambda idx, _phase: make_msg(
            yawspeed=0.8 if idx < seqlen // 2 else -0.8
        )
    )
    append_sequence(
        lambda _idx, phase: make_msg(
            vx=1.2 * phase,
            vy=0.8 * phase,
            yawspeed=0.6,
        )
    )
    append_sequence(
        lambda _idx, phase: make_msg(
            vx=2.0 * min(phase * 2.0, 1.0),
            vy=0.4,
        )
    )
    append_sequence(
        lambda idx, _phase: make_msg(
            vx=1.8 if idx < seqlen // 2 else -1.8,
            yawspeed=0.9 if idx < seqlen // 2 else -0.9,
        )
    )
    append_sequence(
        lambda idx, _phase: make_msg(
            vz=-0.9 if idx < seqlen // 3 else (
                0.9 if idx < (2 * seqlen) // 3 else 0.0
            )
        )
    )
    append_sequence(
        lambda _idx, phase: make_msg(
            vx=1.6 * phase,
            vy=1.2 * phase,
            yawspeed=1.1,
        )
    )
    append_sequence(lambda _idx, _phase: make_msg(yawspeed=1.2))
    append_sequence(
        lambda _idx, phase: make_msg(
            vx=1.4 * math.cos(2.0 * math.pi * phase),
            vy=1.4 * math.sin(2.0 * math.pi * phase),
            yawspeed=1.0,
        )
    )
    append_sequence(
        lambda idx, _phase: make_msg(
            vx=1.0,
            vy=0.8,
            vz=-0.8 if idx % 4 < 2 else 0.8,
        )
    )

    return seeds


def get_init_manual_control_msg():
    if ManualControlSetpoint is None:
        raise RuntimeError("px4_msgs/msg/ManualControlSetpoint is unavailable")
    msg = ManualControlSetpoint()

    # corresponds to joystick position
    msg.x = 0.0 # pitch [-1:1]
    msg.y = 0.0 # roll [-1:1]
    msg.z = 0.5 # throttle [0:1] - 0.5 is neutral (hover)
    msg.r = 0.0 # yaw [-1:1]

    return msg


def get_init_parameter_msg():
    msg = Parameter()
    msg.param_name = "MIS_TAKEOFF_ALT"
    msg.value = 2.5

    return msg


def _load_param_dict():
    global param_dict
    if param_dict is None:
        try:
            param_dict = pickle.load(open("px4_prep/params.pkl", "rb"))
        except FileNotFoundError:
            param_dict = {}
    return param_dict


def _set_vector_field(msg, field_name, values):
    if not hasattr(msg, field_name):
        return False
    try:
        setattr(msg, field_name, list(values))
    except (TypeError, AttributeError):
        seq = getattr(msg, field_name)
        for idx, value in enumerate(values):
            seq[idx] = value
    return True


def _set_if_present(msg, field_name, value):
    if hasattr(msg, field_name):
        setattr(msg, field_name, value)


def _px4_v117_state_qos():
    try:
        from rclpy.qos import (
            QoSProfile,
            QoSReliabilityPolicy,
            QoSHistoryPolicy,
            QoSDurabilityPolicy,
        )
    except ImportError:
        return 10
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def _set_position(msg, x, y, z):
    if _set_vector_field(msg, "position", (x, y, z)):
        return
    if hasattr(msg, "x"):
        msg.x = x
        msg.y = y
        msg.z = z


def _set_velocity(msg, vx, vy, vz):
    if _set_vector_field(msg, "velocity", (vx, vy, vz)):
        return
    if hasattr(msg, "vx"):
        msg.vx = vx
        msg.vy = vy
        msg.vz = vz


def _set_acceleration(msg, ax, ay, az):
    if _set_vector_field(msg, "acceleration", (ax, ay, az)):
        return
    if hasattr(msg, "acceleration"):
        msg.acceleration = [ax, ay, az]


def read_trajectory_seed(seedfile):
    """seedfile contains either a msg in a dictionary (single repeated mode)
    or a list of msg dictionaries (interception mode), depending on the desired mode"""

    if not os.path.isfile(seedfile):
        return None

    with open(seedfile, "r") as fp:
        json_content = json.load(fp)

    if type(json_content) is dict:
        msg = TrajectorySetpoint()
        set_message_fields(msg, json_content)

        return msg

    elif type(json_content) is list:
        trajectory_msg_list = []
        for msg_dict in json_content:
            msg = TrajectorySetpoint()
            set_message_fields(msg, msg_dict)
            trajectory_msg_list.append(msg)

        return trajectory_msg_list

    else:
        print("[-] unknown seed type")
        return None


def read_offboard_mission(missionfile):

    if not os.path.isfile(missionfile):
        return None

    with open(missionfile, "r") as fp:
        msg_list = json.load(fp)

    trajectory_msg_list = []
    for msg_dict in msg_list:
        msg = TrajectorySetpoint()
        set_message_fields(msg, msg_dict)
        trajectory_msg_list.append(msg)

    return trajectory_msg_list


def conduct_offboard_mission(bridge, mission):

    bridge.takeoff()

    for msg in mission:
        bridge.send_command(msg)

    bridge.land()
