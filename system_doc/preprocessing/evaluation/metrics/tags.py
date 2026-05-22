"""维度4: 标签质量评估 — 检查语义标签的精度和分布"""

from __future__ import annotations
import re

from ...schema import SpecBlock
from ..report import DimensionResult


# 严格验证规则：比 TAG_RULES 更严格，用于估计精度
# 格式: tag_name → validator(block) -> bool
STRICT_VALIDATORS: dict[str, callable] = {}


def _register_validator(tag: str):
    def decorator(func):
        STRICT_VALIDATORS[tag] = func
        return func
    return decorator


@_register_validator("velocity_constraint")
def _validate_velocity(block: SpecBlock) -> bool:
    """严格验证：名称含 VEL/SPEED/AIRSPD 且有约束语义"""
    name_ok = bool(re.search(r"(VEL|SPEED|AIRSPD)", block.name, re.I))
    unit_ok = block.structured_fields.get("units", "") in ("m/s", "km/h", "kn", "cm/s")
    unit_ok2 = block.structured_fields.get("unit", "") in ("m/s", "km/h", "kn", "cm/s")
    return name_ok or unit_ok or unit_ok2


@_register_validator("attitude_constraint")
def _validate_attitude(block: SpecBlock) -> bool:
    name_ok = bool(re.search(r"(ROLL|PITCH|YAW|ATT|TILT|MAN_[RPY])", block.name, re.I))
    unit_ok = block.structured_fields.get("units", "") in ("rad", "deg", "rad/s", "deg/s")
    unit_ok2 = block.structured_fields.get("unit", "") in ("rad", "deg", "rad/s", "deg/s")
    return name_ok or unit_ok or unit_ok2


@_register_validator("flight_mode")
def _validate_flight_mode(block: SpecBlock) -> bool:
    name_ok = bool(re.search(r"(FLTMODE|OFFBOARD|POSCTL|ALTCTL|STABILIZED|MANUAL_CONTROL)", block.name, re.I))
    desc_ok = bool(re.search(r"(flight\s*mode|offboard|manual\s*control)", block.natural_language, re.I))
    return name_ok or desc_ok


@_register_validator("safety")
def _validate_safety(block: SpecBlock) -> bool:
    name_ok = bool(re.search(r"(FAILSAFE|FS_|FAIL_ACT|EMERG|ABORT|CBRK)", block.name, re.I))
    desc_ok = bool(re.search(r"(failsafe|emergency|abort|circuit\s*breaker)", block.natural_language, re.I))
    return name_ok or desc_ok


def evaluate_tags(
    blocks: list[SpecBlock],
    over_trigger_threshold: float = 0.30,
    precision_threshold: float = 0.70,
) -> DimensionResult:
    """
    评估标签质量。

    检查项：
    1. 分布分析：是否有标签覆盖率 > over_trigger_threshold
    2. 精度估计：用严格验证规则检查标签是否准确
    3. 未标记比例：没有任何标签的块占比
    """
    total = len(blocks)
    if total == 0:
        return DimensionResult(name="Tag Quality", passed=False, score=0.0, metrics={})

    # 统计标签分布
    tag_counts: dict[str, int] = {}
    untagged = 0
    for block in blocks:
        if not block.tags:
            untagged += 1
        for tag in block.tags:
            tag_counts[tag] = tag_counts.get(tag, 0) + 1

    # 检查过度触发
    over_triggered = {
        tag: count / total
        for tag, count in tag_counts.items()
        if count / total > over_trigger_threshold
    }

    # 精度估计（对有严格验证器的标签）
    tag_precision: dict[str, float] = {}
    for tag, validator in STRICT_VALIDATORS.items():
        tagged_blocks = [b for b in blocks if tag in b.tags]
        if not tagged_blocks:
            continue
        correct = sum(1 for b in tagged_blocks if validator(b))
        tag_precision[tag] = correct / len(tagged_blocks)

    # 综合得分
    avg_precision = sum(tag_precision.values()) / len(tag_precision) if tag_precision else 0.5
    over_trigger_penalty = min(len(over_triggered) * 0.1, 0.3)
    score = max(0.0, avg_precision - over_trigger_penalty)

    metrics = {
        "tag_distribution": {k: f"{v}/{total} ({v/total:.1%})" for k, v in sorted(tag_counts.items(), key=lambda x: -x[1])},
        "tag_precision": {k: f"{v:.1%}" for k, v in tag_precision.items()},
        "over_triggered": {k: f"{v:.1%}" for k, v in over_triggered.items()},
        "untagged_ratio": untagged / total,
        "untagged_count": untagged,
    }

    warnings = [f"Tag '{tag}' over-triggered at {ratio:.1%}" for tag, ratio in over_triggered.items()]

    return DimensionResult(
        name="Tag Quality",
        passed=score >= precision_threshold and len(over_triggered) == 0,
        score=score,
        metrics=metrics,
        failures=[{"tag": t, "coverage": f"{r:.1%}"} for t, r in over_triggered.items()],
        warnings=warnings,
    )
