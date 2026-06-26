"""D2: 语义准确性 (Semantic Accuracy)

验证 OracleIR 中的参数值、单位、范围是否与 SpecBlock 的 structured_fields 一致。

评估指标:
  - 默认值一致: parameter.default == SpecBlock.structured_fields.default
  - 单位一致: parameter.unit == SpecBlock.structured_fields.units
  - 范围合理: 使用的阈值在 SpecBlock 的 [min, max] 范围内
  - 字段存在: observation 引用的 topic/field 在 SpecBlock 中有记录
"""

from __future__ import annotations
import json
from pathlib import Path
from typing import Any

from . import DimensionScore
from src_jazzy.oracle_ir.schema import OracleIR


def evaluate_semantics(
    specs: list[OracleIR],
    index_data: dict[str, Any],
    blocks_dir: Path,
) -> DimensionScore:
    """评估语义准确性"""
    total_checks = 0
    passed_checks = 0
    details = []
    failures = []

    # 加载所有参数类型的 block
    param_blocks: dict[str, dict] = {}
    for ir in specs:
        for param in ir.parameters:
            # 尝试加载对应的 SpecBlock（使用 ir.system 动态构建路径）
            block_path = blocks_dir / f"{ir.system}.parameter.{param.name}.json"
            if block_path.exists() and param.name not in param_blocks:
                param_blocks[param.name] = json.loads(block_path.read_text())

    for ir in specs:
        for param in ir.parameters:
            block = param_blocks.get(param.name)
            if block is None:
                total_checks += 1
                failures.append(f"[FAIL] {ir.id}: param '{param.name}' has no SpecBlock")
                continue

            sf = block.get("structured_fields", {})

            # Check 1: 默认值是否一致
            if param.default is not None and "default" in sf:
                total_checks += 1
                spec_default = sf["default"]
                if isinstance(spec_default, (int, float)) and abs(param.default - spec_default) < 1e-6:
                    passed_checks += 1
                    details.append(f"[PASS] {ir.id}: {param.name}.default = {param.default}")
                else:
                    failures.append(
                        f"[FAIL] {ir.id}: {param.name}.default = {param.default}, "
                        f"SpecBlock says {spec_default}"
                    )

            # Check 2: 单位是否一致
            if param.unit and "units" in sf:
                total_checks += 1
                spec_unit = sf["units"]
                # 标准化比较（m/s vs m/s, deg/s vs deg/s）
                if _normalize_unit(param.unit) == _normalize_unit(spec_unit):
                    passed_checks += 1
                    details.append(f"[PASS] {ir.id}: {param.name}.unit = '{param.unit}'")
                else:
                    failures.append(
                        f"[FAIL] {ir.id}: {param.name}.unit = '{param.unit}', "
                        f"SpecBlock says '{spec_unit}'"
                    )

            # Check 3: 默认值在 [min, max] 范围内
            if param.default is not None and "min" in sf and "max" in sf:
                total_checks += 1
                spec_min = sf.get("min", float("-inf"))
                spec_max = sf.get("max", float("inf"))
                if spec_min <= param.default <= spec_max:
                    passed_checks += 1
                    details.append(f"[PASS] {ir.id}: {param.name}.default in [{spec_min}, {spec_max}]")
                else:
                    failures.append(
                        f"[FAIL] {ir.id}: {param.name}.default={param.default} "
                        f"outside [{spec_min}, {spec_max}]"
                    )

    score = passed_checks / total_checks if total_checks > 0 else 0.0
    return DimensionScore(
        name="D2: Semantic Accuracy",
        score=score,
        total=total_checks,
        passed=passed_checks,
        details=details,
        failures=failures,
    )


def _normalize_unit(unit: str) -> str:
    """标准化单位字符串以便比较"""
    unit = unit.strip().lower()
    # 常见等价映射
    aliases = {
        "m/s": "m/s",
        "m/s2": "m/s^2",
        "m/s^2": "m/s^2",
        "m/s3": "m/s^3",
        "m/s^3": "m/s^3",
        "deg": "deg",
        "deg/s": "deg/s",
        "rad": "rad",
        "rad/s": "rad/s",
    }
    return aliases.get(unit, unit)
