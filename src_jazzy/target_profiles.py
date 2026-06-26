import json
import os
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional

import yaml


KNOWN_PROFILES = {
    "turtlebot4_jazzy",
    "px4_v117_jazzy",
    "moveit2_jazzy",
}


@dataclass
class TargetProfile:
    name: str
    family: str
    raw: dict
    input_topic: Optional[str]
    input_type: Optional[str]
    input_topics: Dict[str, str]
    watch_topics: Dict[str, str]
    optional_watch_topics: Dict[str, str] = field(default_factory=dict)
    topic_aliases: Dict[str, str] = field(default_factory=dict)
    required_actions_for_readiness: Dict[str, str] = field(
        default_factory=dict
    )
    required_log_patterns_for_readiness: List[str] = field(
        default_factory=list
    )
    required_topics_with_data_for_readiness: Dict[str, str] = field(
        default_factory=dict
    )
    launch_adapter: str = ""
    launch_command: List[str] = field(default_factory=list)
    oracle_module: str = ""
    oracle_mode: str = ""
    status: str = ""

    @property
    def required_topics_for_readiness(self) -> Dict[str, str]:
        topics = dict(self.watch_topics)
        if self.family in {"turtlebot", "px4"}:
            topics.update(self.input_topics)
        return topics

    def to_metadata(self) -> dict:
        data = asdict(self)
        data["required_topics_for_readiness"] = self.required_topics_for_readiness
        return data


def repo_root_from_src() -> str:
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def resolve_profile_name(
    target_profile=None,
    tb4=False,
    px4_v117=False,
    test_moveit=False,
    legacy_tb3=False,
):
    if target_profile:
        return target_profile
    if tb4:
        return "turtlebot4_jazzy"
    if px4_v117:
        return "px4_v117_jazzy"
    if test_moveit:
        return "moveit2_jazzy"
    if legacy_tb3:
        return None
    return None


def load_profile(name: str, repo_root: Optional[str] = None) -> TargetProfile:
    if name not in KNOWN_PROFILES:
        raise ValueError(
            f"unknown target profile {name!r}; expected one of "
            f"{sorted(KNOWN_PROFILES)}"
        )

    root = repo_root or repo_root_from_src()
    path = os.path.join(root, "target_profiles", f"{name}.yaml")
    with open(path, "r", encoding="utf-8") as fp:
        raw = yaml.safe_load(fp)

    _validate_profile(raw, path)

    family = raw["target"]["family"]
    input_topics = _extract_input_topics(raw)
    input_topic, input_type = _select_primary_input(family, input_topics, raw)
    required_watch = dict(raw.get("watch", {}).get("required", {}) or {})
    optional_watch = _extract_optional_watch_topics(raw)
    topic_aliases = _default_topic_aliases(family)
    readiness = raw.get("readiness", {}) or {}
    required_actions = dict(
        readiness.get("required_actions", {}) or {}
    )
    required_log_patterns = list(
        readiness.get("required_log_patterns", []) or []
    )
    required_topics_with_data = dict(
        readiness.get("required_topics_with_data", {}) or {}
    )
    launch = raw.get("launch", {}) or {}
    oracle = raw.get("oracle", {}) or {}

    return TargetProfile(
        name=raw["name"],
        family=family,
        raw=raw,
        input_topic=input_topic,
        input_type=input_type,
        input_topics=input_topics,
        watch_topics=required_watch,
        optional_watch_topics=optional_watch,
        topic_aliases=topic_aliases,
        required_actions_for_readiness=required_actions,
        required_log_patterns_for_readiness=required_log_patterns,
        required_topics_with_data_for_readiness=required_topics_with_data,
        launch_adapter=launch.get("adapter", ""),
        launch_command=list(launch.get("command", []) or []),
        oracle_module=oracle.get("module", ""),
        oracle_mode=oracle.get("mode", ""),
        status=raw.get("status", ""),
    )


def write_profile_metadata(profile: TargetProfile, meta_dir: str):
    os.makedirs(meta_dir, exist_ok=True)
    profile_path = os.path.join(meta_dir, "target_profile.resolved.json")
    watchlist_path = os.path.join(meta_dir, "watchlist.resolved.json")

    with open(profile_path, "w", encoding="utf-8") as fp:
        json.dump(profile.to_metadata(), fp, indent=2, sort_keys=True)

    watchlist = dict(profile.watch_topics)
    watchlist.update(profile.optional_watch_topics)
    with open(watchlist_path, "w", encoding="utf-8") as fp:
        json.dump(watchlist, fp, indent=2, sort_keys=True)

    return profile_path, watchlist_path


def attach_profile_to_config(config, profile: TargetProfile):
    config.target_profile = profile
    config.target_profile_name = profile.name
    config.target_family = profile.family
    config.target_generation = profile.raw.get("target", {}).get(
        "generation", ""
    )
    config.input_topic = profile.input_topic
    config.input_type = profile.input_type
    config.input_topics = dict(profile.input_topics)
    config.watch_topics = dict(profile.watch_topics)
    config.optional_watch_topics = dict(profile.optional_watch_topics)
    config.topic_aliases = dict(profile.topic_aliases)
    config.required_actions_for_readiness = dict(
        profile.required_actions_for_readiness
    )
    config.required_log_patterns_for_readiness = list(
        profile.required_log_patterns_for_readiness
    )
    config.required_topics_with_data_for_readiness = dict(
        profile.required_topics_with_data_for_readiness
    )
    config.launch_adapter = profile.launch_adapter
    config.launch_command = list(profile.launch_command)
    config.oracle_mode = profile.oracle_mode

    if profile.family == "turtlebot":
        config.tb4_sitl = True
        config.tb3_sitl = False
        config.tb3_hitl = False
    elif profile.family == "px4":
        config.px4_sitl = True
        config.px4_ros = True
        config.use_mavlink = False
        config.exp_pgfuzz = False
        config.flight_mode = "OFFBOARD"
    elif profile.family == "moveit":
        config.test_moveit = True
        config.moveit_planning_only = False


def normalize_state_dict(state_dict: dict, aliases: Optional[Dict[str, str]]):
    if not aliases:
        return state_dict

    normalized = dict(state_dict)
    for current_topic, legacy_topic in aliases.items():
        if current_topic in state_dict and legacy_topic not in normalized:
            normalized[legacy_topic] = state_dict[current_topic]
    return normalized


def _validate_profile(raw: dict, path: str):
    required = ["name", "target", "launch", "input", "watch", "oracle"]
    missing = [key for key in required if key not in raw]
    if missing:
        raise ValueError(f"{path} is missing required keys: {missing}")
    if "family" not in raw["target"]:
        raise ValueError(f"{path} is missing target.family")


def _extract_input_topics(raw: dict) -> Dict[str, str]:
    input_cfg = raw.get("input", {}) or {}
    if "topics" in input_cfg:
        return dict(input_cfg["topics"] or {})
    if "preferred" in input_cfg:
        preferred = input_cfg["preferred"] or {}
        return {preferred["topic"]: preferred["type"]}
    if "topic" in input_cfg:
        return {input_cfg["topic"]: input_cfg["type"]}
    return {}


def _select_primary_input(family: str, input_topics: Dict[str, str], raw: dict):
    input_cfg = raw.get("input", {}) or {}
    if family == "px4" and "/fmu/in/trajectory_setpoint" in input_topics:
        return (
            "/fmu/in/trajectory_setpoint",
            input_topics["/fmu/in/trajectory_setpoint"],
        )
    if "preferred" in input_cfg:
        preferred = input_cfg["preferred"] or {}
        return preferred.get("topic"), preferred.get("type")
    if input_topics:
        topic = next(iter(input_topics))
        return topic, input_topics[topic]
    return None, None


def _extract_optional_watch_topics(raw: dict) -> Dict[str, str]:
    watch = raw.get("watch", {}) or {}
    optional = {}
    for key in ("optional", "oracle_candidates", "expected_after_planning"):
        optional.update(watch.get(key, {}) or {})
    return optional


def _default_topic_aliases(family: str) -> Dict[str, str]:
    if family == "moveit":
        return {
            "/panda_arm_controller/controller_state":
                "/panda_arm_controller/state",
        }
    if family == "px4":
        return {
            "/fmu/out/vehicle_status_v1": "/VehicleStatus_PubSubTopic",
            "/fmu/out/vehicle_local_position_v1":
                "/VehicleLocalPosition_PubSubTopic",
            "/fmu/out/vehicle_attitude": "/VehicleAttitude_PubSubTopic",
            "/fmu/out/vehicle_odometry": "/VehicleOdometry_PubSubTopic",
            "/fmu/out/vehicle_control_mode": "/VehicleControlMode_PubSubTopic",
            "/fmu/out/battery_status_v1": "/BatteryStatus_PubSubTopic",
            "/fmu/out/sensor_combined": "/SensorCombined_PubSubTopic",
        }
    return {}
