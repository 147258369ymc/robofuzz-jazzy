"""
ULG State Parser — reads PX4 internal .ulg flight logs via pyulog
and produces the same state_dict interface as RosbagParser.

This bypasses the microRTPS bridge entirely, providing ground-truth
PX4 internal state for oracle checks.

Usage:
    parser = UlgStateParser(ulg_path)
    if not parser.abort:
        state_dict = parser.process_messages()
        # state_dict has the same format as RosbagParser output:
        # {"/TopicName_PubSubTopic": [(timestamp, msg_proxy), ...]}
"""

import os
import glob
import math
from pyulog import ULog


# Mapping from .ulg topic names to the ROS PubSub topic names
# used by the oracle (state_dict keys).
ULG_TO_ROSTOPIC = {
    "vehicle_acceleration": "/VehicleAcceleration_PubSubTopic",
    "vehicle_angular_velocity": "/VehicleAngularVelocity_PubSubTopic",
    "vehicle_angular_acceleration": "/VehicleAngularAcceleration_PubSubTopic",
    "vehicle_attitude": "/VehicleAttitude_PubSubTopic",
    "vehicle_local_position": "/VehicleLocalPosition_PubSubTopic",
    "vehicle_odometry": "/VehicleOdometry_PubSubTopic",
    "sensors_status_imu": "/SensorsStatusImu_PubSubTopic",
    "vehicle_gps_position": "/VehicleGpsPosition_PubSubTopic",
    "vehicle_global_position": "/VehicleGlobalPosition_PubSubTopic",
    "actuator_outputs": "/ActuatorOutputs_PubSubTopic",
    "actuator_armed": "/ActuatorArmed_PubSubTopic",
    "vehicle_status": "/VehicleStatus_PubSubTopic",
    "manual_control_setpoint": "/ManualControlSetpoint_PubSubTopic",
}

# PX4 log root inside the container
PX4_LOG_ROOT = (
    "/robofuzz/targets/PX4-Autopilot/build/px4_sitl_rtps"
    "/tmp/rootfs/log"
)


class UlgArrayProxy:
    """Handles msg.xyz[i] / msg.q[i] / msg.output[i] style access."""

    __slots__ = ["_data", "_idx", "_prefix"]

    def __init__(self, data_dict, index, prefix):
        self._data = data_dict
        self._idx = index
        self._prefix = prefix

    def __getitem__(self, i):
        if isinstance(i, slice):
            start, stop, step = i.indices(len(self))
            return [self[j] for j in range(start, stop, step or 1)]
        key = f"{self._prefix}[{i}]"
        if key in self._data:
            return float(self._data[key][self._idx])
        raise KeyError(f"Field '{key}' not found in ULG data")

    def __iter__(self):
        """Allow iteration (e.g., for val in msg.xyz)."""
        i = 0
        while True:
            key = f"{self._prefix}[{i}]"
            if key not in self._data:
                break
            yield float(self._data[key][self._idx])
            i += 1

    def __len__(self):
        i = 0
        while f"{self._prefix}[{i}]" in self._data:
            i += 1
        return i


class UlgMsgProxy:
    """
    Proxy object that mimics ROS msg attribute access from .ulg numpy data.

    The oracle accesses fields as:
        msg.q[0], msg.xyz[2], msg.x, msg.vx, msg.output[3], etc.

    This proxy transparently maps those accesses to the underlying
    numpy arrays from pyulog, indexed at a specific sample position.
    """

    __slots__ = ["_data", "_idx", "_array_cache"]

    def __init__(self, data_dict, index):
        object.__setattr__(self, "_data", data_dict)
        object.__setattr__(self, "_idx", index)
        object.__setattr__(self, "_array_cache", {})

    def __getattr__(self, name):
        data = object.__getattribute__(self, "_data")
        idx = object.__getattribute__(self, "_idx")

        # Direct scalar field (e.g., msg.x, msg.vx, msg.armed)
        if name in data:
            return float(data[name][idx])

        # Array field (e.g., msg.q → UlgArrayProxy for q[0], q[1], ...)
        # Check if any field starts with "name["
        test_key = f"{name}[0]"
        if test_key in data:
            return UlgArrayProxy(data, idx, name)

        raise AttributeError(
            f"UlgMsgProxy has no field '{name}'. "
            f"Available: {[k for k in data.keys() if '[' not in k]}"
        )


class UlgStateParser:
    """
    Parses a PX4 .ulg file and produces a state_dict compatible with
    the oracle's expected input format.

    Drop-in replacement for RosbagParser in the fuzzer pipeline.
    """

    def __init__(self, ulg_path):
        self.ulg_path = ulg_path
        self.abort = False
        self._ulog = None

        if not os.path.isfile(ulg_path):
            print(f"[-] ULG file not found: {ulg_path}")
            self.abort = True
            return

        try:
            self._ulog = ULog(ulg_path)
        except Exception as e:
            print(f"[-] Failed to parse ULG: {e}")
            self.abort = True

    def _get_takeoff_time(self):
        """Find the timestamp when the vehicle leaves the ground."""
        for d in self._ulog.data_list:
            if d.name == "vehicle_land_detected":
                ts = d.data["timestamp"]
                landed = d.data["landed"]
                for i in range(len(landed)):
                    if landed[i] == 0:
                        return ts[i]
        # Fallback: use arm time
        for d in self._ulog.data_list:
            if d.name == "actuator_armed":
                ts = d.data["timestamp"]
                armed = d.data["armed"]
                for i in range(len(armed)):
                    if armed[i] == 1:
                        return ts[i]
        return 0

    def process_messages(self):
        """
        Parse the .ulg and return state_dict in the same format as
        RosbagParser.process_messages():

            {"/TopicName_PubSubTopic": [(timestamp_ns, UlgMsgProxy), ...]}

        Only includes data from after takeoff (filters ground phase).
        Timestamps are converted to nanoseconds to match rosbag convention.
        """
        if self.abort or self._ulog is None:
            return {}

        takeoff_us = self._get_takeoff_time()
        state_dict = {}

        for d in self._ulog.data_list:
            ros_topic = ULG_TO_ROSTOPIC.get(d.name)
            if ros_topic is None:
                continue

            # Skip duplicate instances (use only the first/primary)
            if ros_topic in state_dict:
                continue

            ts_array = d.data["timestamp"]
            samples = []

            for i in range(len(ts_array)):
                ts_us = ts_array[i]
                if ts_us < takeoff_us:
                    continue
                # Convert microseconds to nanoseconds (rosbag convention)
                ts_ns = int(ts_us * 1000)
                proxy = UlgMsgProxy(d.data, i)
                samples.append((ts_ns, proxy))

            if samples:
                state_dict[ros_topic] = samples

        return state_dict

    def process_all_messages(self):
        """
        Parse all messages without time filtering.
        Fallback method matching RosbagParser.process_all_messages().
        """
        if self.abort or self._ulog is None:
            return {}

        state_dict = {}

        for d in self._ulog.data_list:
            ros_topic = ULG_TO_ROSTOPIC.get(d.name)
            if ros_topic is None:
                continue
            if ros_topic in state_dict:
                continue

            ts_array = d.data["timestamp"]
            samples = []
            for i in range(len(ts_array)):
                ts_ns = int(ts_array[i] * 1000)
                proxy = UlgMsgProxy(d.data, i)
                samples.append((ts_ns, proxy))

            if samples:
                state_dict[ros_topic] = samples

        return state_dict


# ===================================================================
# Utility functions for .ulg file management
# ===================================================================

def find_latest_ulg(log_root=PX4_LOG_ROOT):
    """
    Find the most recently modified .ulg file under the PX4 log root.
    Returns the file path, or None if no .ulg files exist.
    """
    pattern = os.path.join(log_root, "**", "*.ulg")
    files = glob.glob(pattern, recursive=True)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def cleanup_ulg_files(log_root=PX4_LOG_ROOT, keep_latest=1):
    """
    Remove old .ulg files, keeping only the N most recent ones.
    Call this after each oracle check to prevent disk fill.
    """
    pattern = os.path.join(log_root, "**", "*.ulg")
    files = glob.glob(pattern, recursive=True)
    if len(files) <= keep_latest:
        return
    files.sort(key=os.path.getmtime, reverse=True)
    for f in files[keep_latest:]:
        try:
            os.remove(f)
        except OSError:
            pass


def preserve_ulg_on_error(ulg_path, dest_dir, frame_id):
    """
    Copy the .ulg file to the experiment log directory when an error
    is detected, preserving evidence for post-analysis.
    """
    import shutil

    ulg_dir = os.path.join(dest_dir, "ulg")
    os.makedirs(ulg_dir, exist_ok=True)
    dest_path = os.path.join(ulg_dir, f"{frame_id}.ulg")
    try:
        shutil.copy2(ulg_path, dest_path)
    except (OSError, shutil.SameFileError) as e:
        print(f"[!] Failed to preserve ULG: {e}")
