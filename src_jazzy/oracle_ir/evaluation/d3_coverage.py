"""D3: 覆盖完整性 (Coverage Completeness)

验证 SpecIndex 中标记为约束类的 SpecBlock 是否被 OracleIR 覆盖。

核心思路:
  SpecIndex 中带有 velocity_constraint / attitude_constraint / altitude_constraint
  等标签的参数，如果其 shortDesc 包含 "max" / "limit" / "bound" 等关键词，
  则认为它是一个"应该被 oracle 覆盖的约束参数"。

评估指标:
  - Recall: 约束参数中有多少被 OracleIR 引用了
  - 遗漏列表: 哪些约束参数没有被任何 OracleIR 覆盖
"""

from __future__ import annotations
import json
import re
from pathlib import Path
from typing import Any

from . import DimensionScore
from src_jazzy.oracle_ir.schema import OracleIR


# 约束相关的标签
CONSTRAINT_TAGS = {
    "velocity_constraint",
    "attitude_constraint",
    "altitude_constraint",
}

# 表示"限制"语义的关键词
LIMIT_KEYWORDS = re.compile(
    r"\b(max|maximum|min|minimum|limit|bound|threshold|ceiling|floor|cap)\b", re.I
)

# 安全约束参数的 group 前缀（排除 EKF/估计器内部参数）
SAFETY_GROUPS = re.compile(
    r"(Multicopter Position Control|Multicopter Attitude Control|"
    r"Multicopter Rate Control|Commander|Return Mode|"
    r"Fixed-wing|VTOL|Mission|Geofence|Land)", re.I
)

# 排除的 group（内部算法参数，不是飞行安全边界）
INTERNAL_GROUPS = re.compile(
    r"(EKF2|Sensor|Estimator|Calibration|Serial|MAVLink|"
    r"Logger|UAVCAN|Simulator|Testing)", re.I
)

# 排除的参数名前缀（arming 检查、电池等非飞行动态约束）
EXCLUDE_PREFIXES = re.compile(
    r"^(COM_ARM_|BAT_|CBRK_|NAV_ACC_|FW_LND_|RWTO_)", re.I
)


def evaluate_coverage(
    specs: list[OracleIR],
    index_data: dict[str, Any],
    blocks_dir: Path,
) -> DimensionScore:
    """评估覆盖完整性"""
    details = []
    failures = []

    # 从 specs 中推断目标系统名称
    system_prefix = ""
    if specs:
        system_prefix = f"{specs[0].system}.parameter."

    # Step 1: 从 SpecIndex 中提取"应该被覆盖的约束参数"
    tag_index = index_data.get("tag_index", {})
    constraint_block_ids: set[str] = set()
    for tag in CONSTRAINT_TAGS:
        constraint_block_ids.update(tag_index.get(tag, []))

    # 过滤: 只保留 parameter 类型且 shortDesc 含限制语义的
    should_cover: dict[str, str] = {}  # param_name → shortDesc
    for bid in constraint_block_ids:
        # 动态匹配：使用推断的系统前缀，或回退到通用 ".parameter." 检测
        if system_prefix and not bid.startswith(system_prefix):
            continue
        if not system_prefix and ".parameter." not in bid:
            continue
        # 去除后缀 _1, _2 (多源重复)
        block_path = blocks_dir / f"{bid}.json"
        if not block_path.exists():
            continue
        block = json.loads(block_path.read_text())
        sf = block.get("structured_fields", {})
        short_desc = sf.get("shortDesc", "")
        name = sf.get("name", "")
        # 只关注含"限制"语义的参数，且属于安全相关 group
        if LIMIT_KEYWORDS.search(short_desc) and name:
            group = sf.get("group", "")
            # 排除内部算法参数
            if INTERNAL_GROUPS.search(group):
                continue
            # 排除非飞行动态约束
            if EXCLUDE_PREFIXES.search(name):
                continue
            should_cover[name] = short_desc

    # Step 2: 收集 OracleIR 引用的所有参数名
    covered_params: set[str] = set()
    for ir in specs:
        for param in ir.parameters:
            covered_params.add(param.name)

    # Step 3: 计算覆盖率
    total = len(should_cover)
    covered = covered_params & set(should_cover.keys())
    missed = set(should_cover.keys()) - covered_params

    for name in sorted(covered):
        details.append(f"[COVERED] {name}: {should_cover[name]}")
    for name in sorted(missed):
        failures.append(f"[MISSED] {name}: {should_cover[name]}")

    score = len(covered) / total if total > 0 else 0.0
    return DimensionScore(
        name="D3: Coverage Completeness",
        score=score,
        total=total,
        passed=len(covered),
        details=details,
        failures=failures,
    )
