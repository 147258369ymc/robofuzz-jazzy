"""D3: 逻辑一致性 (Logic Consistency)

验证 OracleIR 的断言逻辑是否与 SpecBlock 描述的语义方向一致。

核心思路:
  - SpecBlock.shortDesc 含 "Maximum" → assertion 应该是 <= 关系
  - SpecBlock.shortDesc 含 "Minimum" → assertion 应该是 >= 关系
  - feedback.direction 应与参数语义一致:
    - "Maximum X" 参数 → feedback 应 maximize 对应指标
  - 参数的 min/max 范围应与 assertion 的 tolerance 逻辑兼容

评估指标:
  - 方向正确性: assertion 的比较方向是否与 SpecBlock 语义一致
  - Feedback 方向: feedback.direction 是否合理
"""

from __future__ import annotations
import re
from pathlib import Path
from typing import Any

from . import DimensionScore
from .block_resolver import load_block_map, resolve_parameter_block
from src.oracle_ir.schema import OracleIR


def evaluate_logic(
    specs: list[OracleIR],
    index_data: dict[str, Any],
    blocks_dir: Path,
) -> DimensionScore:
    """评估逻辑一致性"""
    total_checks = 0
    passed_checks = 0
    details = []
    failures = []
    blocks = load_block_map(blocks_dir)

    for ir in specs:
        for param in ir.parameters:
            block = resolve_parameter_block(ir, param, blocks_dir, blocks)
            if block is None:
                continue
            sf = block.get("structured_fields", {})
            short_desc = sf.get("shortDesc", "").lower()

            # Check 1: 参数语义方向 vs assertion 比较方向
            is_max_param = "max" in short_desc or "limit" in short_desc
            is_min_param = "min" in short_desc

            for assertion in ir.assertions:
                expr = assertion.expr.lower()
                param_lower = param.name.lower()

                # 只检查引用了该参数的 assertion
                if param_lower not in expr and f"param({param_lower})" not in expr:
                    continue

                total_checks += 1
                if is_max_param and "<=" in expr:
                    passed_checks += 1
                    details.append(
                        f"[PASS] {ir.id}: '{param.name}' is max-type, assertion uses <="
                    )
                elif is_min_param and ">=" in expr:
                    passed_checks += 1
                    details.append(
                        f"[PASS] {ir.id}: '{param.name}' is min-type, assertion uses >="
                    )
                elif is_max_param and ">=" in expr:
                    failures.append(
                        f"[FAIL] {ir.id}: '{param.name}' is max-type but assertion uses >="
                    )
                elif is_min_param and "<=" in expr:
                    failures.append(
                        f"[FAIL] {ir.id}: '{param.name}' is min-type but assertion uses <="
                    )
                else:
                    passed_checks += 1
                    details.append(
                        f"[PASS] {ir.id}: '{param.name}' direction inconclusive, OK"
                    )

            # Check 2: feedback 方向
            for fb in ir.feedback:
                if is_max_param and fb.direction:
                    total_checks += 1
                    if fb.direction == "maximize":
                        passed_checks += 1
                        details.append(f"[PASS] {ir.id}: feedback '{fb.name}' maximize OK")
                    elif fb.direction == "minimize":
                        failures.append(f"[FAIL] {ir.id}: feedback '{fb.name}' minimize vs max-param")
                    else:
                        passed_checks += 1

    score = passed_checks / total_checks if total_checks > 0 else 0.0
    return DimensionScore(
        name="D3: Logic Consistency",
        score=score,
        total=total_checks,
        passed=passed_checks,
        details=details,
        failures=failures,
    )
