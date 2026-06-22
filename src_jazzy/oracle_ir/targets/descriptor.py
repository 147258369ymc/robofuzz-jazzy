"""
目标系统描述符 (Target Descriptor)

这是实现多目标可迁移性的核心抽象。
每个目标系统（PX4、TurtleBot、机械臂、机器狗）提供一个 descriptor，
描述它的通信接口、状态空间、操作模式和物理约束。

Agent 和 Compiler 通过 descriptor 获取目标特定信息，
而不是在代码里硬编码 PX4 的概念。
"""

from __future__ import annotations
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
import json
import yaml


@dataclass
class TopicDescriptor:
    """一个通信 topic 的描述"""
    name: str                  # topic 名称，如 "/cmd_vel", "/joint_states"
    msg_type: str              # 消息类型，如 "geometry_msgs/Twist"
    fields: list[str]          # 可用字段列表
    field_units: dict[str, str] = field(default_factory=dict)  # field → unit
    frequency_hz: float = 0.0  # 典型发布频率
    description: str = ""


@dataclass
class OperatingMode:
    """操作模式（替代 PX4 的 flight_modes）"""
    name: str                  # 如 "manual", "autonomous", "teleoperation"
    description: str = ""
    # 该模式下哪些 topic 是活跃的
    active_topics: list[str] = field(default_factory=list)


@dataclass
class PhysicalConstraint:
    """物理约束模板 — 描述该系统天然存在的约束类型"""
    category: str              # velocity | position | force | torque | angle | temperature
    description: str = ""
    # 该类约束通常关联哪些 topics
    relevant_topics: list[str] = field(default_factory=list)
    # 该类约束通常关联哪些参数名模式（正则）
    param_patterns: list[str] = field(default_factory=list)


@dataclass
class SafetyCondition:
    """安全前置条件（替代 require_airborne）"""
    name: str                  # 如 "is_moving", "is_armed", "gripper_closed"
    description: str = ""
    # 判断表达式（引用 topic.field）
    check_expr: str = ""
    # 需要订阅的 topic
    required_topics: list[str] = field(default_factory=list)


@dataclass
class TargetDescriptor:
    """
    目标系统完整描述符。

    这是整个可迁移性设计的核心：
    - 预处理的 TAG_RULES 从这里生成
    - Agent 的 system prompt 从这里生成
    - Compiler 的 scope 过滤从这里获取条件
    - GENERATION_TEMPLATE 的 topic 列表从这里获取
    """
    # 基本信息
    name: str                          # "px4", "turtlebot3", "franka_arm", "unitree_go2"
    display_name: str                  # "PX4 Multicopter", "Franka Emika Panda"
    version: str = ""
    middleware: str = "ros2"           # ros2 | ros1 | custom
    description: str = ""

    # 通信接口
    topics: list[TopicDescriptor] = field(default_factory=list)

    # 操作模式（通用化的 flight_modes）
    operating_modes: list[OperatingMode] = field(default_factory=list)

    # 物理约束类型
    constraint_types: list[PhysicalConstraint] = field(default_factory=list)

    # 安全前置条件（通用化的 require_airborne）
    safety_conditions: list[SafetyCondition] = field(default_factory=list)

    # 参数命名约定（用于自动标签生成）
    param_naming: dict[str, list[str]] = field(default_factory=dict)
    # 格式: {"velocity_constraint": [".*VEL.*", ".*SPEED.*"], ...}

    # topic 命名模板（不同系统的 topic 命名规则不同）
    topic_suffix: str = ""             # PX4 用 "_PubSubTopic"，ROS2 通常无后缀

    def get_watchlist(self) -> dict[str, str]:
        """生成 topic → msg_type 的 watchlist（给 Validator 用）"""
        return {t.name: t.msg_type for t in self.topics}

    def get_tag_rules(self) -> list[tuple[str, str]]:
        """
        生成自动标签规则（给 IndexBuilder 用）。
        返回 [(regex_pattern, tag_name), ...]
        """
        rules = []
        for category, patterns in self.param_naming.items():
            for pat in patterns:
                rules.append((pat, category))
        return rules

    def get_scope_template(self) -> dict:
        """生成 scope 模板（给 Agent prompt 用）"""
        return {
            "operating_modes": [m.name for m in self.operating_modes],
            "safety_conditions": {
                sc.name: sc.check_expr for sc in self.safety_conditions
            },
        }

    def to_agent_context(self) -> str:
        """生成给 Agent 的系统描述文本"""
        lines = [
            f"目标系统: {self.display_name}",
            f"通信中间件: {self.middleware}",
            f"",
            f"## 可用 Topics ({len(self.topics)} 个)",
        ]
        for t in self.topics:
            fields_str = ", ".join(t.fields[:8])
            lines.append(f"  - {t.name} ({t.msg_type})")
            lines.append(f"    fields: [{fields_str}]")
            if t.field_units:
                units_str = ", ".join(f"{k}={v}" for k, v in list(t.field_units.items())[:5])
                lines.append(f"    units: {units_str}")

        lines.append(f"")
        lines.append(f"## 操作模式")
        for m in self.operating_modes:
            lines.append(f"  - {m.name}: {m.description}")

        lines.append(f"")
        lines.append(f"## 安全前置条件（替代 scope.filter_expr）")
        for sc in self.safety_conditions:
            lines.append(f"  - {sc.name}: {sc.check_expr}")
            lines.append(f"    说明: {sc.description}")

        return "\n".join(lines)

    def save(self, path: Path):
        """保存为 YAML 文件"""
        from dataclasses import asdict
        data = asdict(self)
        path.write_text(
            yaml.dump(data, allow_unicode=True, default_flow_style=False),
            encoding="utf-8",
        )

    @classmethod
    def load(cls, path: Path) -> "TargetDescriptor":
        """从 YAML 文件加载"""
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        # 递归重建 dataclass
        data["topics"] = [TopicDescriptor(**t) for t in data.get("topics", [])]
        data["operating_modes"] = [OperatingMode(**m) for m in data.get("operating_modes", [])]
        data["constraint_types"] = [PhysicalConstraint(**c) for c in data.get("constraint_types", [])]
        data["safety_conditions"] = [SafetyCondition(**s) for s in data.get("safety_conditions", [])]
        return cls(**data)
