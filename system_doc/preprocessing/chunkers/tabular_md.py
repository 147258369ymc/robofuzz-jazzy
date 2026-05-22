"""表格型 Markdown 分块器 — 按消息/命令/参数表格段分块"""

from __future__ import annotations
import re
from pathlib import Path

from .base import BaseChunker, RawChunk


# 匹配 Markdown 表格行
_TABLE_ROW_RE = re.compile(r"^\s*\|(.+)\|\s*$")
# 匹配分隔行
_TABLE_SEP_RE = re.compile(r"^\s*\|[\s\-:|]+\|\s*$")
# 匹配 heading
_HEADING_RE = re.compile(r"^(#{1,6})\s+(.+)$")


class TableMarkdownChunker(BaseChunker):
    """
    将含表格的 Markdown 按"heading + 其下表格"为单位分块。
    每个 heading 段（含其表格）作为一个 RawChunk。
    对于纯表格文档（如参数参考），按表格行分块。
    """

    def __init__(self, row_level_chunking: bool = False):
        """
        Args:
            row_level_chunking: True 时按表格行分块（适合参数列表），
                               False 时按 heading 段分块（适合消息/命令定义）
        """
        self.row_level_chunking = row_level_chunking

    @property
    def supported_doc_type(self) -> str:
        return "tabular_markdown"

    def chunk(self, content: str, file_path: Path) -> list[RawChunk]:
        if self.row_level_chunking:
            return self._chunk_by_rows(content, file_path)
        return self._chunk_by_sections(content, file_path)

    def _chunk_by_rows(self, content: str, file_path: Path) -> list[RawChunk]:
        """按表格行分块，适合参数列表等每行一个实体的文档"""
        lines = content.splitlines()
        chunks = []
        current_heading = ""
        header_cells: list[str] = []
        idx = 0

        for i, line in enumerate(lines):
            heading_match = _HEADING_RE.match(line)
            if heading_match:
                current_heading = heading_match.group(2).strip()
                header_cells = []
                continue

            if _TABLE_SEP_RE.match(line):
                continue

            row_match = _TABLE_ROW_RE.match(line)
            if row_match:
                cells = [c.strip() for c in row_match.group(1).split("|")]
                if not header_cells:
                    header_cells = cells
                    continue
                # 数据行 → 一个 chunk
                row_dict = {}
                for j, h in enumerate(header_cells):
                    if j < len(cells):
                        row_dict[h] = cells[j]
                name = cells[0] if cells else f"row_{idx}"
                chunks.append(RawChunk(
                    content=line,
                    name=name,
                    chunk_type=self._infer_row_type(row_dict),
                    index=idx,
                    parent_heading=current_heading,
                    location=f"L{i + 1}",
                    extra=row_dict,
                ))
                idx += 1
        return chunks

    def _chunk_by_sections(self, content: str, file_path: Path) -> list[RawChunk]:
        """按 heading 段分块，每段包含 heading + 描述 + 表格"""
        lines = content.splitlines()
        sections: list[dict] = []
        current: dict | None = None

        for i, line in enumerate(lines):
            heading_match = _HEADING_RE.match(line)
            if heading_match:
                if current:
                    sections.append(current)
                current = {
                    "heading": heading_match.group(2).strip(),
                    "level": len(heading_match.group(1)),
                    "start_line": i + 1,
                    "lines": [line],
                }
            elif current:
                current["lines"].append(line)

        if current:
            sections.append(current)

        # 只保留含表格的 section（level >= 2）
        chunks = []
        parent_heading = ""
        for idx, sec in enumerate(sections):
            if sec["level"] == 1:
                parent_heading = sec["heading"]
                continue
            text = "\n".join(sec["lines"])
            has_table = any(_TABLE_ROW_RE.match(l) for l in sec["lines"])
            if not has_table and len(sec["lines"]) < 3:
                continue
            chunk_type = self._infer_section_type(sec["heading"], text)
            chunks.append(RawChunk(
                content=text,
                name=sec["heading"],
                chunk_type=chunk_type,
                index=len(chunks),
                parent_heading=parent_heading,
                location=f"L{sec['start_line']}",
                extra={"level": sec["level"], "has_table": has_table},
            ))
        return chunks

    def _infer_section_type(self, heading: str, text: str) -> str:
        h_lower = heading.lower()
        if any(k in h_lower for k in ("command", "cmd")):
            return "command"
        if any(k in h_lower for k in ("message", "msg", "heartbeat", "status")):
            return "message"
        if any(k in h_lower for k in ("param", "parameter", "config")):
            return "parameter"
        if any(k in h_lower for k in ("field",)):
            return "field"
        return "message"

    def _infer_row_type(self, row_dict: dict) -> str:
        keys_lower = {k.lower() for k in row_dict.keys()}
        if {"min", "max", "default"} & keys_lower:
            return "parameter"
        if {"type", "unit", "description"} & keys_lower:
            return "field"
        if {"id", "command"} & keys_lower:
            return "command"
        return "data_entry"
