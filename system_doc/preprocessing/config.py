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
    version="moveit2-2.2.3_panda-2.0.3",
    doc_root="system_doc/moveit2_panda",
    # 不指定 file_configs → 自动递归发现所有文件并按扩展名分派分块器
    # 目录结构:
    #   config/         — URDF, SRDF, YAML 配置（robot_model + yaml_params 分块器）
    #   msg_definitions/ — MoveIt2 msg/srv/action（ros_interface 分块器）
    #   ros2_interfaces/ — 标准 ROS2 消息定义（ros_interface 分块器）
    #   source_constraints/ — 测试目标 C++ 源码（source_code 分块器）
    #   official_specs/  — 整理的硬件/安全规范 Markdown（tabular_markdown / heading_md 分块器）
)


def get_config(target_name: str) -> TargetConfig:
    """获取目标系统配置"""
    configs = {
        "px4": PX4_CONFIG,
        "turtlebot3": TURTLEBOT3_CONFIG,
        "moveit2_panda": MOVEIT2_PANDA_CONFIG,
    }
    if target_name not in configs:
        raise ValueError(
            f"Unknown target '{target_name}'. Available: {list(configs.keys())}"
        )
    return configs[target_name]
