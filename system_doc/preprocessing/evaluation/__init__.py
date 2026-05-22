"""预处理评估框架 — 自动化评估流水线输出质量"""

from .runner import EvaluationRunner
from .report import EvaluationReport, DimensionResult

__all__ = ["EvaluationRunner", "EvaluationReport", "DimensionResult"]
