"""评估报告数据结构"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any


@dataclass
class DimensionResult:
    """单个评估维度的结果"""
    name: str
    passed: bool                       # 是否达到阈值
    score: float                       # 0.0 ~ 1.0 综合得分
    metrics: dict[str, Any]            # 具体指标
    failures: list[dict[str, Any]] = field(default_factory=list)  # 具体失败案例
    warnings: list[str] = field(default_factory=list)

    def summary_line(self) -> str:
        status = "PASS" if self.passed else "FAIL"
        return f"[{status}] {self.name}: {self.score:.1%}"


@dataclass
class EvaluationReport:
    """完整评估报告"""
    target: str
    version: str
    total_blocks: int
    dimensions: list[DimensionResult] = field(default_factory=list)

    @property
    def overall_score(self) -> float:
        if not self.dimensions:
            return 0.0
        return sum(d.score for d in self.dimensions) / len(self.dimensions)

    @property
    def all_passed(self) -> bool:
        return all(d.passed for d in self.dimensions)

    def format_report(self) -> str:
        lines = [
            "=" * 50,
            "  Preprocessing Evaluation Report",
            "=" * 50,
            f"Target: {self.target} (v{self.version})",
            f"Total blocks: {self.total_blocks}",
            f"Overall score: {self.overall_score:.1%}",
            "",
        ]

        for i, dim in enumerate(self.dimensions, 1):
            status = "PASS" if dim.passed else "FAIL"
            lines.append(f"[{i}/{len(self.dimensions)}] {dim.name} [{status}] — {dim.score:.1%}")

            for key, val in dim.metrics.items():
                if isinstance(val, float):
                    lines.append(f"  {key}: {val:.1%}")
                elif isinstance(val, dict):
                    lines.append(f"  {key}:")
                    for k2, v2 in list(val.items())[:10]:
                        lines.append(f"    {k2}: {v2}")
                else:
                    lines.append(f"  {key}: {val}")

            if dim.failures:
                lines.append(f"  Failures ({len(dim.failures)} total):")
                for f in dim.failures[:5]:
                    lines.append(f"    - {f}")
            if dim.warnings:
                for w in dim.warnings[:3]:
                    lines.append(f"  WARNING: {w}")
            lines.append("")

        lines.append("=" * 50)
        return "\n".join(lines)
