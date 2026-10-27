"""
目标系统配置 — 定义不同目标系统的预处理参数。
新增目标时只需添加一个 TargetConfig 实例。
"""

from __future__ import annotations
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class FileConfig:
    """单个文件的预处理配置"""
    path: str                          # 相对于 doc_root 的路径
    chunker_override: str | None = None  # 强制使用某种分块器
    chunker_params: dict[str, Any] = field(default_factory=dict)
    skip: bool = False                 # 是否跳过此文件


@dataclass
class TargetConfig:
    """目标系统预处理配置"""
    name: str                          # 系统名称: "px4", "turtlebot3"
    version: str                       # 版本标识
    doc_root: str                      # 文档根目录
    file_configs: list[FileConfig] = field(default_factory=list)
    # 全局参数
    json_array_key: str | None = None  # JSON 文件默认的数组 key
    xml_element_tag: str | None = None # XML 文件默认的元素标签


# === 预定义配置 ===

PX4_CONFIG = TargetConfig(
    name="px4",
    version="a264541861",
    doc_root="system_doc/px4",
    json_array_key="parameters",
    xml_element_tag="parameter",
    file_configs=[
        FileConfig("parameters.json"),
        # parameters.xml 与 parameters.json 内容完全相同（同一构建产物的两种格式），跳过避免冗余
        FileConfig("parameters.xml", skip=True),
        # parameter_reference_v1.12.md 是同批参数的 Markdown 渲染，跳过避免三重冗余
        FileConfig(
            "parameter_reference_v1.12.md",
            chunker_override="tabular_markdown",
            chunker_params={"row_level_chunking": True},
            skip=True,
        ),
        FileConfig(
            "uorb_message_reference.md",
            chunker_override="tabular_markdown",
            chunker_params={"row_level_chunking": True},
        ),
        FileConfig(
            "vehicle_command_reference.md",
            chunker_override="tabular_markdown",
            chunker_params={"row_level_chunking": True},
        ),
        FileConfig("mavlink_common_messages.md"),
        FileConfig(
            "mavlink_parameter_protocol.md",
            chunker_override="protocol_spec",
        ),
        FileConfig("parameters_and_configurations.md"),
        FileConfig("README.md", skip=True),
    ],
)


# TurtleBot3 配置：使用精简后的 turtlebot3_clean 目录，包含所有格式的文件。
TURTLEBOT3_CONFIG = TargetConfig(
    name="turtlebot3",
    version="foxy-2.1.1",
    doc_root="system_doc/turtlebot3_clean",
    # 不指定 file_configs → 自动递归发现所有文件并按扩展名分派分块器
)


# MoveIt2 + Franka Emika Panda 配置
MOVEIT2_PANDA_CONFIG = TargetConfig(
    name="moveit2_panda",
    version="moveit2-2.12.4_panda-3.1.0_jazzy",
    doc_root="system_doc/moveit2_panda",
    # 不指定 file_configs → 自动递归发现所有文件并按扩展名分派分块器
    # 目录结构:
    #   config/         — URDF, SRDF, YAML 配置（robot_model + yaml_params 分块器）
    #   msg_definitions/ — MoveIt2 msg/srv/action（ros_interface 分块器）
    #   ros2_interfaces/ — 标准 ROS2 消息定义（ros_interface 分块器）
    #   source_constraints/ — 测试目标 C++ 源码（source_code 分块器）
    #   official_specs/  — 整理的硬件/安全规范 Markdown（tabular_markdown / heading_md 分块器）
)


TURTLEBOT4_JAZZY_CONFIG = TargetConfig(
    name="turtlebot4_jazzy",
    version="turtlebot4-jazzy-runtime-clean",
    doc_root="system_doc/turtlebot4_clean",
    # 不指定 file_configs → 自动递归发现官方文档、runtime 参数、ROS 接口、
    # launch/world/profile 约束和 ready metadata。目标特定语义由
    # src/oracle_ir/targets/turtlebot4_jazzy.yaml 的 descriptor 提供。
)


PX4_V117_JAZZY_CONFIG = TargetConfig(
    name="px4_v117_jazzy",
    version="px4-v1.17.0_jazzy_gz_x500_uxrce_dds",
    doc_root="system_doc/px4_v117_jazzy",
    # PX4 v1.17 的 corpus 会保留完整官方文档和源约束，但默认预处理
    # 使用 oracleIR-focused 文件集，避免同一接口在 px4_msgs、
    # PX4 source、ros2 interface show 三处重复进入索引。
    file_configs=[
        FileConfig("extracted_specs/*.md"),
        FileConfig("known_issues.md"),
        FileConfig("versioned_runtime_sources.md"),
        FileConfig("official_specs/offboard.md"),
        FileConfig("official_specs/ros2_offboard_control.md"),
        FileConfig("official_specs/uxrce_dds.md"),
        FileConfig("official_specs/sim_gazebo_gz_index.md"),
        FileConfig("official_specs/sim_gazebo_gz_vehicles.md"),
        FileConfig("official_specs/msg_docs/*.md"),
        FileConfig("msg_definitions/px4_msgs/*.msg"),
        FileConfig(
            "params/parameter_reference_v1.17_compact.md",
            chunker_override="tabular_markdown",
            chunker_params={"row_level_chunking": True},
        ),
        FileConfig("params/selected_parameter_blocks.md"),
        FileConfig("source_constraints/bridge/dds_topics.yaml"),
        FileConfig("source_constraints/airframes/4001_gz_x500"),
        FileConfig("source_constraints/profile/px4_v117_jazzy.yaml"),
        FileConfig("runtime_metadata/container_versions.tsv"),
    ],
)


SLAM_TOOLBOX_JAZZY_CONFIG = TargetConfig(
    name="slam_toolbox_jazzy",
    version="slam_toolbox-2.8.5_turtlebot4-navigation-2.1.1_jazzy",
    doc_root="system_doc/slam_toolbox_jazzy",
    # 只索引当前同步在线 mapping 的高信号语料。锁定的完整实现证据保存在
    # source_archive/，但不默认送入 OracleIR，防止 async/localization/
    # lifelong/map-merging 等非运行模式污染生成上下文。
    file_configs=[
        FileConfig("extracted_specs/*.md"),
        FileConfig("known_issues.md"),
        FileConfig("versioned_runtime_sources.md"),
        FileConfig("official_specs/*.md"),
        FileConfig("params/robofuzz.slam_toolbox_jazzy.params.yaml"),
        FileConfig("ros2_interfaces/*.msg"),
        FileConfig("ros2_interfaces/*.srv"),
        FileConfig("slam_toolbox_interfaces/srv/Pause.srv"),
        FileConfig("slam_toolbox_interfaces/srv/ClearQueue.srv"),
        FileConfig("slam_toolbox_interfaces/srv/Reset.srv"),
        FileConfig("slam_toolbox_interfaces/srv/SaveMap.srv"),
        FileConfig("source_constraints/profile/slam_toolbox_jazzy.yaml"),
        FileConfig("source_constraints/oracle/slam.py"),
        FileConfig("source_constraints/qos/*.yaml"),
        FileConfig("source_constraints/world/*.sdf"),
        FileConfig("runtime_metadata/container_versions.tsv"),
        FileConfig("runtime_metadata/baseline_observations.md"),
    ],
)


NAV2_JAZZY_CONFIG = TargetConfig(
    name="nav2_jazzy",
    version="nav2-1.3.12_turtlebot4_jazzy",
    doc_root="system_doc/nav2_jazzy",
    # 目标驱动、高信噪比语料:只索引 NavigateToPose 目标导航 + 真值定位 + 4 判据
    # 实际依赖的组件约束。原始上游 official_specs/*.rst 保留在磁盘作为优先级-2
    # 证据,但不默认送入 OracleIR —— 其语义已蒸馏进 extracted_specs/*.md,直接喂
    # 原始 .rst(如 mppi 1291 行)会污染生成上下文(与 slam_toolbox_jazzy 同策略)。
    file_configs=[
        # 蒸馏的 oracle 导向契约(送生成器主料)。强制 prose_markdown:这些契约含
        # 大量 "timeout"/"sequence"/"flow" 等词,会被 detector 误判为 protocol_spec
        # 而分块失败(liveness 契约实测 0 blocks)。显式覆盖保证按 heading 稳定分块。
        FileConfig("extracted_specs/arrival_correctness_contract.md",
                   chunker_override="prose_markdown"),
        FileConfig("extracted_specs/collision_freedom_contract.md",
                   chunker_override="prose_markdown"),
        FileConfig("extracted_specs/liveness_timeout_stuck_contract.md",
                   chunker_override="prose_markdown"),
        FileConfig("extracted_specs/false_failure_contract.md",
                   chunker_override="prose_markdown"),
        FileConfig("extracted_specs/runtime_and_observability_contract.md",
                   chunker_override="prose_markdown"),
        FileConfig("known_issues.md", chunker_override="prose_markdown"),
        FileConfig("versioned_runtime_sources.md"),
        # 镜像实测参数默认值(判据阈值锚点,逐参数分块)
        FileConfig("params/nav2_bringup.nav2_params.yaml"),
        FileConfig("params/turtlebot4_navigation.nav2.yaml"),
        # 干净的 action 接口定义(Goal/Result/Feedback 三段)
        FileConfig("official_specs/nav2_msgs_NavigateToPose.action"),
        FileConfig("official_specs/nav2_msgs_ComputePathToPose.action"),
        FileConfig("official_specs/nav2_msgs_FollowPath.action"),
        # GoalStatus 状态码常量(判据 3/4 的 SUCCEEDED=4/CANCELED=5/ABORTED=6)
        FileConfig(
            "ros2_interfaces/action_msgs_msg_GoalStatus.txt",
            chunker_override="ros_interface",
        ),
        # 当前 RoboFuzz 实现快照
        FileConfig("source_constraints/profile/nav2_jazzy.yaml"),
        # nav2.py 含模块级 NAV2_* 阈值常量(oracle 判据阈值锚点)。nav2_map.py /
        # generate_nav2_goal_seeds.py / nav2_gt_tf.py 是纯逻辑(类/函数,无模块级
        # 常量),SourceCodeChunker 只提取 UPPER_CASE 常量 → 0 blocks,故不纳入
        # (与 slam 只喂 oracle/slam.py 同策略;逻辑证据仍在磁盘 source_constraints)。
        FileConfig("source_constraints/oracle/nav2.py"),
        # 运行时版本证据
        FileConfig("runtime_metadata/container_versions.tsv"),
    ],
)


def get_config(target_name: str) -> TargetConfig:
    """获取目标系统配置"""
    configs = {
        "px4": PX4_CONFIG,
        "turtlebot3": TURTLEBOT3_CONFIG,
        "moveit2_panda": MOVEIT2_PANDA_CONFIG,
        "turtlebot4_jazzy": TURTLEBOT4_JAZZY_CONFIG,
        "px4_v117_jazzy": PX4_V117_JAZZY_CONFIG,
        "slam_toolbox_jazzy": SLAM_TOOLBOX_JAZZY_CONFIG,
        "nav2_jazzy": NAV2_JAZZY_CONFIG,
    }
    if target_name not in configs:
        raise ValueError(
            f"Unknown target '{target_name}'. Available: {list(configs.keys())}"
        )
    return configs[target_name]
