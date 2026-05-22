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
        FileConfig("parameters.xml"),
        FileConfig(
            "parameter_reference_v1.12.md",
            chunker_override="tabular_markdown",
            chunker_params={"row_level_chunking": True},
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


# TurtleBot3 配置模板（示例，展示可迁移性）
TURTLEBOT3_CONFIG = TargetConfig(
    name="turtlebot3",
    version="humble",
    doc_root="system_doc/turtlebot3",
    file_configs=[
        # 添加 TurtleBot3 文档时在此配置
    ],
)


def get_config(target_name: str) -> TargetConfig:
    """获取目标系统配置"""
    configs = {
        "px4": PX4_CONFIG,
        "turtlebot3": TURTLEBOT3_CONFIG,
    }
    if target_name not in configs:
        raise ValueError(
            f"Unknown target '{target_name}'. Available: {list(configs.keys())}"
        )
    return configs[target_name]
