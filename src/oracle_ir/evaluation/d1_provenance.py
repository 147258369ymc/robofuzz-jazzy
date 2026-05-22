"""D1: 溯源正确性 (Provenance Accuracy)

验证 OracleIR 的 provenance 引用是否指向真实存在的 SpecBlock，
且引用的 SpecBlock 与 OracleIR 的语义类型相关。

评估指标:
  - 存在性: chunk_id 在 SpecIndex 中是否可解析
  - 相关性: 引用的 SpecBlock 的 tags 是否与 OracleIR type 对齐
  - 完整性: 每个 assertion 是否至少有一个 provenance 支撑
"""

from __future__ import annotations
from pathlib import Path
from typing import Any

from . import DimensionScore
from src.oracle_ir.schema import OracleIR


# OracleIR type → 期望的 SpecBlock tags 映射（宽松匹配）
TYPE_TAG_ALIGNMENT = {
    "range_bound": {"velocity_constraint", "attitude_constraint", "altitude_constraint", "actuator"},
    "validity": {"sensor", "velocity_constraint", "attitude_constraint"},
    "norm_constraint": {"attitude_constraint", "position_constraint"},
    "temporal_consistency": {"velocity_constraint", "attitude_constraint", "sensor", "position_constraint"},
    "cross_sensor": {"sensor", "position_constraint", "velocity_constraint"},
}


def evaluate_provenance(
    specs: list[OracleIR],
    index_data: dict[str, Any],
) -> DimensionScore:
    """评估溯源正确性"""
    total_checks = 0
    passed_checks = 0
    details = []
    failures = []

    entity_index = index_data.get("entity_index", {})
    tag_index = index_data.get("tag_index", {})

    # 构建 block_id → tags 的反向映射
    block_tags: dict[str, set[str]] = {}
    for tag, block_ids in tag_index.items():
        for bid in block_ids:
            block_tags.setdefault(bid, set()).add(tag)

    # 构建所有已知 block_id 集合
    all_block_ids = set()
    for ids in entity_index.values():
        all_block_ids.update(ids)
    for ids in tag_index.values():
        all_block_ids.update(ids)

    for ir in specs:
        # Check 1: 每个 OracleIR 至少有一个 provenance
        total_checks += 1
        if ir.provenance:
            passed_checks += 1
            details.append(f"[PASS] {ir.id}: has {len(ir.provenance)} provenance refs")
        else:
            failures.append(f"[FAIL] {ir.id}: no provenance references")

        # Check 2: 每个 chunk_id 是否存在于索引中
        for prov in ir.provenance:
            if not prov.chunk_id:
                continue
            total_checks += 1
            # 尝试精确匹配或前缀匹配
            found = prov.chunk_id in all_block_ids
            if not found:
                # 尝试模糊匹配: "px4.param.X" → "px4.parameter.X"
                alt_id = prov.chunk_id.replace(".param.", ".parameter.")
                alt_id2 = prov.chunk_id.replace(".msg.", ".field.")
                found = alt_id in all_block_ids or alt_id2 in all_block_ids
            if found:
                passed_checks += 1
                details.append(f"[PASS] {ir.id}: chunk_id '{prov.chunk_id}' resolvable")
            else:
                failures.append(f"[FAIL] {ir.id}: chunk_id '{prov.chunk_id}' NOT in index")

        # Check 3: tag 对齐 — provenance 引用的 block 的 tags 是否与 oracle type 相关
        expected_tags = TYPE_TAG_ALIGNMENT.get(ir.type, set())
        if expected_tags and ir.provenance:
            total_checks += 1
            matched = False
            for prov in ir.provenance:
                bid = prov.chunk_id
                # 尝试规范化
                for candidate in [bid, bid.replace(".param.", ".parameter."), bid.replace(".msg.", ".field.")]:
                    btags = block_tags.get(candidate, set())
                    if btags & expected_tags:
                        matched = True
                        break
            if matched:
                passed_checks += 1
                details.append(f"[PASS] {ir.id}: tag alignment OK")
            else:
                failures.append(f"[FAIL] {ir.id}: type '{ir.type}' expects tags {expected_tags}, none found in provenance")

    score = passed_checks / total_checks if total_checks > 0 else 0.0
    return DimensionScore(
        name="D1: Provenance Accuracy",
        score=score,
        total=total_checks,
        passed=passed_checks,
        details=details,
        failures=failures,
    )
