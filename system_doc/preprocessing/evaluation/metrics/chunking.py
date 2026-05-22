"""维度1: 分块完整性评估 — 检查是否有实体在分块过程中丢失"""

from __future__ import annotations
from pathlib import Path

from ..report import DimensionResult
from ..ground_truth import GroundTruthEntry


def evaluate_chunking(
    ground_truth: dict[str, list[GroundTruthEntry]],
    block_names: set[str],
    threshold: float = 0.95,
) -> DimensionResult:
    """
    评估分块完整性。

    Args:
        ground_truth: {source_file: [GroundTruthEntry, ...]}
        block_names: 所有输出 block 的 name 集合
        threshold: 通过阈值（recall >= threshold 视为 PASS）
    """
    metrics = {}
    all_missing = []
    total_expected = 0
    total_found = 0

    for source_file, entries in ground_truth.items():
        expected_names = {e.name for e in entries}
        found = expected_names & block_names
        missing = expected_names - block_names

        recall = len(found) / len(expected_names) if expected_names else 1.0
        file_key = Path(source_file).name
        metrics[f"recall_{file_key}"] = recall
        metrics[f"count_{file_key}"] = f"{len(found)}/{len(expected_names)}"

        total_expected += len(expected_names)
        total_found += len(found)

        for name in sorted(missing)[:10]:
            all_missing.append({"entity": name, "source": file_key})

    overall_recall = total_found / total_expected if total_expected else 1.0
    metrics["overall_recall"] = overall_recall
    metrics["total_expected"] = total_expected
    metrics["total_found"] = total_found

    return DimensionResult(
        name="Chunking Recall",
        passed=overall_recall >= threshold,
        score=overall_recall,
        metrics=metrics,
        failures=all_missing,
        warnings=[f"Missing {total_expected - total_found} entities"] if total_found < total_expected else [],
    )
