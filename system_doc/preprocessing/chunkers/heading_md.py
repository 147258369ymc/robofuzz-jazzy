"""叙述型 Markdown 分块器 — 按 heading 层级分块"""

from __future__ import annotations
import re
from pathlib import Path

from .base import BaseChunker, RawChunk


_HEADING_RE = re.compile(r"^(#{1,6})\s+(.+)$")


class HeadingMarkdownChunker(BaseChunker):
    """
    按 heading 层级切分叙述型 Markdown。
    每个 level-2+ heading 及其内容作为一个块。
    适合架构说明、API 文档、使用指南等。
    """

    def __init__(self, min_level: int = 2, max_level: int = 4):
        """
        Args:
            min_level: 开始分块的 heading 层级（默认从 ## 开始）
            max_level: 最深分块层级（更深的归入父块）
        """
        self.min_level = min_level
        self.max_level = max_level

    @property
    def supported_doc_type(self) -> str:
        return "prose_markdown"

    def chunk(self, content: str, file_path: Path) -> list[RawChunk]:
        lines = content.splitlines()
        chunks = []
        current_section: dict | None = None
        parent_stack: list[str] = []

        for i, line in enumerate(lines):
            m = _HEADING_RE.match(line)
            if m:
                level = len(m.group(1))
                title = m.group(2).strip()

                # 更新 parent_stack
                while parent_stack and len(parent_stack) >= level:
                    parent_stack.pop()
                parent_heading = " > ".join(parent_stack) if parent_stack else ""

                if self.min_level <= level <= self.max_level:
                    # 保存前一个 section
                    if current_section:
                        chunks.append(self._make_chunk(current_section, len(chunks)))
                    current_section = {
                        "heading": title,
                        "level": level,
                        "start_line": i + 1,
                        "lines": [line],
                        "parent_heading": parent_heading,
                    }
                elif level < self.min_level:
                    # 顶层 heading，保存前一个并更新 parent
                    if current_section:
                        chunks.append(self._make_chunk(current_section, len(chunks)))
                        current_section = None
                elif current_section:
                    # 更深层级归入当前块
                    current_section["lines"].append(line)

                parent_stack.append(title)
            elif current_section:
                current_section["lines"].append(line)

        if current_section:
            chunks.append(self._make_chunk(current_section, len(chunks)))
        return chunks

    def _make_chunk(self, section: dict, index: int) -> RawChunk:
        text = "\n".join(section["lines"])
        return RawChunk(
            content=text,
            name=section["heading"],
            chunk_type="config_desc",
            index=index,
            parent_heading=section["parent_heading"],
            location=f"L{section['start_line']}",
            extra={"level": section["level"]},
        )
