"""文档类型探测器 — 自动判断文档格式并选择对应分块策略"""

from __future__ import annotations
from pathlib import Path
from enum import Enum
import re


class DocType(Enum):
    STRUCTURED_DATA = "structured_data"        # JSON/XML/YAML
    TABULAR_MARKDOWN = "tabular_markdown"      # 含大量表格的 Markdown
    PROSE_MARKDOWN = "prose_markdown"           # 叙述性 Markdown
    PROTOCOL_SPEC = "protocol_spec"            # 协议时序/流程描述


# 协议关键词
_PROTOCOL_KEYWORDS = re.compile(
    r"(sequence|request\s*→?\s*response|handshake|timeout|retransmit|state\s+machine|"
    r"flow|re-?request|protocol\s+operation)",
    re.IGNORECASE,
)


class DocTypeDetector:
    """
    基于文件扩展名和内容特征自动探测文档类型。
    可通过 register_rule() 扩展探测规则以适配新目标。
    """

    def __init__(self):
        self._custom_rules: list[tuple[callable, DocType]] = []

    def register_rule(self, predicate: callable, doc_type: DocType):
        """注册自定义探测规则: predicate(path, content) -> bool"""
        self._custom_rules.append((predicate, doc_type))

    def detect(self, file_path: Path, content: str | None = None) -> DocType:
        if content is None:
            content = file_path.read_text(encoding="utf-8")

        # 自定义规则优先
        for predicate, dtype in self._custom_rules:
            if predicate(file_path, content):
                return dtype

        # 扩展名判断
        suffix = file_path.suffix.lower()
        if suffix in (".json", ".xml", ".yaml", ".yml"):
            return DocType.STRUCTURED_DATA

        # Markdown 细分
        if suffix in (".md", ".markdown"):
            return self._classify_markdown(content)

        # 未知格式回退到 prose
        return DocType.PROSE_MARKDOWN

    def _classify_markdown(self, content: str) -> DocType:
        lines = content.splitlines()
        if not lines:
            return DocType.PROSE_MARKDOWN

        total = len(lines)
        table_lines = sum(1 for l in lines if "|" in l and l.strip().startswith("|"))
        table_ratio = table_lines / total if total > 0 else 0

        # 表格占比 > 50% 优先判定为表格型（即使含协议关键词）
        if table_ratio > 0.50:
            return DocType.TABULAR_MARKDOWN

        # 协议描述特征：含协议关键词 + 时序描述 + 表格占比不高
        protocol_matches = len(_PROTOCOL_KEYWORDS.findall(content))
        if protocol_matches >= 3 and table_ratio < 0.30:
            return DocType.PROTOCOL_SPEC

        # 表格占比 > 30% 判定为表格型
        if table_ratio > 0.30:
            return DocType.TABULAR_MARKDOWN

        return DocType.PROSE_MARKDOWN
