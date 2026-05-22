"""维度2: 归一化准确性评估 — 检查字段提取和名称质量"""

from __future__ import annotations
import re

from ...schema import SpecBlock
from ..report import DimensionResult
from ..ground_truth import GroundTruthEntry


# 判断名称是否有意义的规则
_BAD_NAME_PATTERNS = [
    re.compile(r"^\d+$"),           # 纯数字
    re.compile(r"^row_\d+$"),       # 通用行号
    re.compile(r"^.{0,2}$"),        # 过短（<=2字符）
]


def evaluate_normalization(
    blocks: list[SpecBlock],
    ground_truth_entries: list[GroundTruthEntry],
    threshold: float = 0.90,
) -> DimensionResult:
    """
    评估归一化准确性。

    检查项：
    1. name 质量：是否为有意义的标识符
    2. structured_fields 完整性：对有 ground truth 的块检查字段是否正确
    3. natural_language 覆盖率：非空比例
    """
    total = len(blocks)
    if total == 0:
        return DimensionResult(name="Normalization Accuracy", passed=False, score=0.0, metrics={})

    # 1. Name 质量
    bad_names = []
    for block in blocks:
        for pattern in _BAD_NAME_PATTERNS:
            if pattern.match(block.name):
                bad_names.append({"block_id": block.block_id, "name": block.name})
                break

    name_quality = 1.0 - len(bad_names) / total

    # 2. structured_fields 完整性（对有 extra 的 ground truth 条目）
    gt_by_name = {e.name: e for e in ground_truth_entries if e.extra}
    field_checks = 0
    field_correct = 0

    for block in blocks:
        if block.name not in gt_by_name:
            continue
        gt_entry = gt_by_name[block.name]
        if not gt_entry.extra or not block.structured_fields:
            continue
        # 检查关键字段是否匹配
        for key in ("min", "max", "default", "type", "units", "unit"):
            if key in gt_entry.extra:
                field_checks += 1
                block_val = block.structured_fields.get(key)
                gt_val = gt_entry.extra[key]
                if block_val == gt_val:
                    field_correct += 1
                elif str(block_val) == str(gt_val):
                    field_correct += 1

    field_accuracy = field_correct / field_checks if field_checks else 1.0

    # 3. natural_language 覆盖率
    nl_non_empty = sum(1 for b in blocks if b.natural_language and len(b.natural_language.strip()) > 5)
    nl_coverage = nl_non_empty / total

    # 综合得分
    score = (name_quality * 0.3 + field_accuracy * 0.4 + nl_coverage * 0.3)

    metrics = {
        "name_quality": name_quality,
        "field_accuracy": field_accuracy,
        "field_checks_total": field_checks,
        "nl_coverage": nl_coverage,
        "bad_name_count": len(bad_names),
    }

    return DimensionResult(
        name="Normalization Accuracy",
        passed=score >= threshold,
        score=score,
        metrics=metrics,
        failures=bad_names[:20],
        warnings=[
            f"{len(bad_names)} blocks have generic/meaningless names"
        ] if bad_names else [],
    )
