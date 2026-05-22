"""OracleIR schema — 形式化 Oracle 中间表示的数据结构定义"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any


@dataclass
class Observation:
    """观测变量：从 ROS topic 中读取的状态字段"""
    name: str
    topic: str
    field: str
    unit: str = ""
    index: int | None = None  # 数组字段的索引，如 xyz[0]


@dataclass
class Parameter:
    """参数依赖：来自系统配置的阈值参数"""
    name: str
    source: str = ""       # 如 "px4.parameters"
    unit: str = ""
    default: float | None = None


@dataclass
class Constant:
    """常量：硬编码的物理常数或规格值"""
    name: str
    value: float
    unit: str = ""


@dataclass
class DerivedVar:
    """派生量：从观测变量计算得到的中间值"""
    name: str
    expr: str              # 受限表达式
    unit: str = ""


@dataclass
class Assertion:
    """断言：oracle 的核心检查逻辑"""
    expr: str              # 受限表达式（布尔）
    tolerance: float = 0.0
    severity: str = "error"  # error | warning
    message: str = ""


@dataclass
class Scope:
    """适用条件：oracle 生效的前置条件"""
    flight_modes: list[str] = field(default_factory=list)
    vehicle_type: str = ""
    require_airborne: bool = False
    preconditions: list[str] = field(default_factory=list)


@dataclass
class Window:
    """时间窗口：采样策略"""
    type: str = "every_sample"  # every_sample | sliding | after_event
    size: float = 0.0           # sliding window 大小（秒）
    filter: str = ""            # 过滤条件名


@dataclass
class FeedbackSpec:
    """语义反馈：驱动 fuzzing 引导的指标"""
    name: str
    metric: str            # 受限表达式
    direction: str = "maximize"  # maximize | minimize | zero | target
    min_threshold: float | None = None
    target_value: float | None = None


@dataclass
class ProvenanceRef:
    """来源追溯：指向预处理产出的 SpecBlock"""
    chunk_id: str = ""
    source_file: str = ""
    evidence: str = ""


@dataclass
class OracleIR:
    """完整的 OracleIR 规约实例"""
    id: str
    type: str              # range_bound | validity | norm_constraint | temporal_consistency | cross_sensor | ...
    system: str
    version: str = ""

    scope: Scope = field(default_factory=Scope)
    observations: list[Observation] = field(default_factory=list)
    parameters: list[Parameter] = field(default_factory=list)
    constants: list[Constant] = field(default_factory=list)
    derived: list[DerivedVar] = field(default_factory=list)
    assertions: list[Assertion] = field(default_factory=list)
    window: Window = field(default_factory=Window)
    feedback: list[FeedbackSpec] = field(default_factory=list)
    provenance: list[ProvenanceRef] = field(default_factory=list)

