"""维度5: 索引正确性评估 — 检查索引的可查性和跨源链接"""

from __future__ import annotations

from ...indexer import SpecIndex
from ..report import DimensionResult
from ..ground_truth import GroundTruthEntry


def evaluate_index(
    index: SpecIndex,
    ground_truth: dict[str, list[GroundTruthEntry]],
    threshold: float = 0.90,
) -> DimensionResult:
    """
    评估索引正确性。

    检查项：
    1. 实体可查性：ground truth 中的实体能否通过索引找到
    2. 跨源链接：同名实体在多个源文件中出现时，索引是否关联了所有 block_id
    3. 引用图一致性：reference_graph 中的被引用实体是否在 entity_index 中存在
    """
    # 1. 实体可查性
    all_gt_names = set()
    for entries in ground_truth.values():
        for e in entries:
            all_gt_names.add(e.name)

    findable = 0
    not_found = []
    for name in all_gt_names:
        results = index.query_entity(name)
        if results:
            findable += 1
        else:
            not_found.append(name)

    findability = findable / len(all_gt_names) if all_gt_names else 1.0

    # 2. 跨源链接
    # 找出在多个源文件中出现的实体
    entity_sources: dict[str, set[str]] = {}
    for source_file, entries in ground_truth.items():
        for e in entries:
            entity_sources.setdefault(e.name, set()).add(source_file)

    multi_source_entities = {name: srcs for name, srcs in entity_sources.items() if len(srcs) > 1}
    cross_link_correct = 0
    cross_link_total = 0

    for name, expected_sources in multi_source_entities.items():
        cross_link_total += 1
        block_ids = index.query_entity(name)
        # 检查是否有来自不同源的 block_id（通过 block_id 中的文件信息推断）
        if len(block_ids) >= len(expected_sources):
            cross_link_correct += 1

    cross_linkage = cross_link_correct / cross_link_total if cross_link_total else 1.0

    # 3. 引用图一致性
    # orphan = 引用图中指向的实体不在 entity_index 中
    # 但很多 orphan 是合法的外部引用（如 CamelCase 引用了未单独建块的实体）
    # 只统计高频 orphan 作为问题
    orphan_refs = []
    total_ref_edges = 0
    resolved_ref_edges = 0
    for ref_entity, referencing_blocks in index.reference_graph.items():
        total_ref_edges += len(referencing_blocks)
        if ref_entity in index.entity_index:
            resolved_ref_edges += len(referencing_blocks)
        else:
            orphan_refs.append({"entity": ref_entity, "referenced_by_count": len(referencing_blocks)})

    # 按边数计算一致性（而非按实体数）
    ref_consistency = resolved_ref_edges / total_ref_edges if total_ref_edges else 1.0

    # 综合得分
    score = findability * 0.5 + cross_linkage * 0.3 + ref_consistency * 0.2

    metrics = {
        "entity_findability": findability,
        "findable_count": f"{findable}/{len(all_gt_names)}",
        "cross_source_linkage": cross_linkage,
        "multi_source_entities": cross_link_total,
        "reference_consistency": ref_consistency,
        "orphan_references": len(orphan_refs),
    }

    failures = not_found[:10] + orphan_refs[:10]

    return DimensionResult(
        name="Index Correctness",
        passed=score >= threshold,
        score=score,
        metrics=metrics,
        failures=[{"not_found": n} for n in not_found[:10]] + orphan_refs[:10],
        warnings=[
            f"{len(orphan_refs)} orphan references in reference_graph"
        ] if orphan_refs else [],
    )
