"""维度3: 引用精度评估 — 检查 references 字段的有效性"""

from __future__ import annotations

from ...schema import SpecBlock
from ..report import DimensionResult
from ..stoplist import get_stoplist


def evaluate_references(
    blocks: list[SpecBlock],
    known_entities: set[str],
    target: str = "default",
    threshold: float = 0.70,
) -> DimensionResult:
    """
    评估引用精度。

    Args:
        blocks: 所有 SpecBlock
        known_entities: ground truth 中所有已知实体名的集合
        target: 目标系统名（用于选择停用词表）
        threshold: precision 通过阈值
    """
    stoplist = get_stoplist(target)

    total_refs = 0
    valid_refs = 0
    stoplist_hits = 0
    unknown_refs = 0
    false_positives: dict[str, int] = {}
    unknown_list: dict[str, int] = {}

    for block in blocks:
        for ref in block.references:
            total_refs += 1
            if ref in known_entities:
                valid_refs += 1
            elif ref in stoplist:
                stoplist_hits += 1
                false_positives[ref] = false_positives.get(ref, 0) + 1
            else:
                unknown_refs += 1
                unknown_list[ref] = unknown_list.get(ref, 0) + 1

    # Precision: valid / (total - unknown)
    # unknown refs 可能是合法实体（只是不在 ground truth 中），不算误报
    judged_refs = valid_refs + stoplist_hits
    precision = valid_refs / judged_refs if judged_refs else 1.0

    # 排序误报和 unknown
    top_fps = sorted(false_positives.items(), key=lambda x: -x[1])[:15]
    top_unknown = sorted(unknown_list.items(), key=lambda x: -x[1])[:10]

    metrics = {
        "total_references": total_refs,
        "valid_references": valid_refs,
        "precision": precision,
        "stoplist_hits": stoplist_hits,
        "unknown_refs": unknown_refs,
        "unknown_ratio": unknown_refs / total_refs if total_refs else 0.0,
        "top_false_positives": {k: v for k, v in top_fps},
        "top_unknown": {k: v for k, v in top_unknown},
    }

    failures = [{"reference": k, "count": v, "type": "stoplist"} for k, v in top_fps]

    return DimensionResult(
        name="Reference Precision",
        passed=precision >= threshold,
        score=precision,
        metrics=metrics,
        failures=failures,
        warnings=[
            f"{stoplist_hits} stoplist hits detected"
        ] if stoplist_hits > 0 else [],
    )
