"""OracleIR 评估框架 — 基于 SpecIndex 回溯验证 LLM 生成的规约质量

评估维度:
  D1: 溯源正确性 (Provenance Accuracy) — chunk_id 是否存在且相关
  D2: 语义准确性 (Semantic Accuracy)   — 数值/单位/范围是否与 SpecBlock 一致
  D3: 覆盖完整性 (Coverage Completeness) — 约束类 SpecBlock 是否被 OracleIR 覆盖
  D4: 逻辑一致性 (Logic Consistency)    — 断言方向是否与文档语义匹配
"""

from __future__ import annotations
import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import sys
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.oracle_ir.schema import OracleIR
from src.oracle_ir.transform.parser import load_all_specs


# =========================================================================
# 评估结果数据结构
# =========================================================================

@dataclass
class DimensionScore:
    """单维度评估结果"""
    name: str
    score: float           # 0.0 ~ 1.0
    total: int             # 检查总数
    passed: int            # 通过数
    details: list[str] = field(default_factory=list)  # 详细信息
    failures: list[str] = field(default_factory=list)  # 失败项


@dataclass
class EvalReport:
    """完整评估报告"""
    dimensions: list[DimensionScore] = field(default_factory=list)
    overall_score: float = 0.0

    def compute_overall(self):
        if self.dimensions:
            self.overall_score = sum(d.score for d in self.dimensions) / len(self.dimensions)
