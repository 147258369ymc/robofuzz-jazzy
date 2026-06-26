"""OracleIR 校验器 — Schema 校验 + 接口对齐 + 表达式验证"""

from __future__ import annotations
from dataclasses import dataclass, field
from pathlib import Path
import re
from typing import Any

from ..schema import OracleIR
from .expr_engine import validate_expr


@dataclass
class ValidationResult:
    """校验结果"""
    valid: bool
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    def __bool__(self):
        return self.valid


def validate_oracle_ir(
    ir: OracleIR,
    watchlist: dict[str, str] | None = None,
    known_params: set[str] | None = None,
) -> ValidationResult:
    """
    校验 OracleIR 实例的完整性和正确性。

    Args:
        ir: OracleIR 实例
        watchlist: {topic: msg_type} 映射（可选，用于接口对齐）
        known_params: 已知参数名集合（可选）
    """
    errors = []
    warnings = []

    # 1. 必填字段校验
    if not ir.id:
        errors.append("Missing required field: id")
    if not ir.type:
        errors.append("Missing required field: type")
    if not ir.system:
        errors.append("Missing required field: system")
    if not ir.assertions and not ir.feedback:
        errors.append("Missing required field: assertions or feedback (at least one)")
    if not ir.observations and not ir.constants:
        warnings.append("No observations or constants declared")

    # 2. 观测变量校验
    obs_names = set()
    for obs in ir.observations:
        if not obs.name:
            errors.append("Observation missing name")
        if not obs.topic:
            errors.append(f"Observation '{obs.name}' missing topic")
        if not obs.field:
            errors.append(f"Observation '{obs.name}' missing field")
        if obs.name in obs_names:
            errors.append(f"Duplicate observation name: {obs.name}")
        obs_names.add(obs.name)

    # 3. 接口对齐：topic 是否在 watchlist 中
    if watchlist:
        for obs in ir.observations:
            if obs.topic and obs.topic not in watchlist:
                errors.append(
                    f"Topic '{obs.topic}' not in watchlist "
                    f"(observation: {obs.name})"
                )

    # 4. 参数校验
    param_names = set()
    for p in ir.parameters:
        if not p.name:
            errors.append("Parameter missing name")
        param_names.add(p.name)
        if known_params and p.name not in known_params:
            warnings.append(f"Parameter '{p.name}' not found in known params")

    # 5. 常量校验
    const_names = set()
    for c in ir.constants:
        if not c.name:
            errors.append("Constant missing name")
        const_names.add(c.name)

    # 6. 派生量表达式校验
    all_var_names = obs_names | param_names | const_names
    derived_names = set()
    for d in ir.derived:
        if not d.name:
            errors.append("Derived variable missing name")
        if not d.expr:
            errors.append(f"Derived '{d.name}' missing expr")
        else:
            expr_errors = validate_expr(d.expr)
            for e in expr_errors:
                errors.append(f"Derived '{d.name}' expr error: {e}")
        derived_names.add(d.name)

    all_var_names |= derived_names

    # 7. 断言表达式校验
    for i, assertion in enumerate(ir.assertions):
        if not assertion.expr:
            errors.append(f"Assertion[{i}] missing expr")
        else:
            expr_errors = validate_expr(assertion.expr)
            for e in expr_errors:
                errors.append(f"Assertion[{i}] expr error: {e}")

    # 8. Feedback 校验
    for fb in ir.feedback:
        if not fb.name:
            errors.append("Feedback missing name")
        if not fb.metric:
            errors.append(f"Feedback '{fb.name}' missing metric")
        else:
            expr_errors = validate_expr(fb.metric)
            for e in expr_errors:
                errors.append(f"Feedback '{fb.name}' metric error: {e}")
        if fb.direction not in ("maximize", "minimize", "zero", "target"):
            errors.append(
                f"Feedback '{fb.name}' invalid direction: {fb.direction}"
            )
        if fb.direction == "maximize" and _looks_like_remaining_margin(fb):
            warnings.append(
                f"Feedback '{fb.name}' maximizes a remaining margin; "
                "boundary-seeking fuzz feedback should minimize remaining "
                "margin or maximize observed pressure/ratio"
            )

    # 9. Provenance 校验
    if not ir.provenance:
        warnings.append("No provenance declared (oracle not traceable)")

    return ValidationResult(
        valid=len(errors) == 0,
        errors=errors,
        warnings=warnings,
    )


def _looks_like_remaining_margin(feedback) -> bool:
    """Heuristic warning for target-agnostic boundary feedback direction."""
    name = (feedback.name or "").lower()
    metric = (feedback.metric or "").strip()
    if not metric:
        return False
    if "margin" in name or "shortfall" in name:
        if "-" in metric:
            return True
    return bool(
        re.match(r"^(param\([^)]+\)|[A-Za-z_][A-Za-z0-9_]*)\s*-", metric)
    )
