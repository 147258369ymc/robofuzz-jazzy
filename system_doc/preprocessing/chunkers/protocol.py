"""协议描述分块器 — 按协议交互步骤/状态分块"""

from __future__ import annotations
import re
from pathlib import Path

from .base import BaseChunker, RawChunk


_HEADING_RE = re.compile(r"^(#{1,6})\s+(.+)$")
# 协议步骤标记
_STEP_MARKERS = re.compile(
    r"(^\d+\.\s|^step\s+\d|^phase\s+\d|request|response|timeout|retry|"
    r"re-?request|sequence|handshake)",
    re.IGNORECASE | re.MULTILINE,
)


class ProtocolFlowChunker(BaseChunker):
    """
    按协议交互步骤分块。适合 MAVLink 协议描述、通信时序文档等。
    分块策略：以 heading 为主分界，内部按编号步骤细分。
    """

    @property
    def supported_doc_type(self) -> str:
        return "protocol_spec"

    def chunk(self, content: str, file_path: Path) -> list[RawChunk]:
        lines = content.splitlines()
        chunks = []
        current_section: dict | None = None
        parent_heading = ""

        for i, line in enumerate(lines):
            m = _HEADING_RE.match(line)
            if m:
                if current_section:
                    chunks.extend(self._split_section(current_section, len(chunks)))
                level = len(m.group(1))
                title = m.group(2).strip()
                if level <= 2:
                    parent_heading = title
                    current_section = None
                else:
                    current_section = {
                        "heading": title,
                        "level": level,
                        "start_line": i + 1,
                        "lines": [line],
                        "parent_heading": parent_heading,
                    }
            elif current_section:
                current_section["lines"].append(line)

        if current_section:
            chunks.extend(self._split_section(current_section, len(chunks)))
        return chunks

    def _split_section(self, section: dict, base_index: int) -> list[RawChunk]:
        """尝试将一个 section 按编号步骤细分"""
        text = "\n".join(section["lines"])
        # 如果内容较短或没有明显步骤标记，整体作为一个块
        step_count = len(_STEP_MARKERS.findall(text))
        if step_count < 2 or len(section["lines"]) < 10:
            return [RawChunk(
                content=text,
                name=section["heading"],
                chunk_type="protocol_step",
                index=base_index,
                parent_heading=section["parent_heading"],
                location=f"L{section['start_line']}",
                extra={"step_markers": step_count},
            )]

        # 按编号列表项分割
        sub_chunks = []
        current_step_lines: list[str] = []
        step_name = section["heading"]
        step_start = section["start_line"]

        for line in section["lines"]:
            numbered = re.match(r"^(\d+)\.\s+(.+)", line)
            if numbered and current_step_lines:
                sub_chunks.append(RawChunk(
                    content="\n".join(current_step_lines),
                    name=step_name,
                    chunk_type="protocol_step",
                    index=base_index + len(sub_chunks),
                    parent_heading=section["parent_heading"],
                    location=f"L{step_start}",
                    extra={"step_markers": 1},
                ))
                step_name = f"{section['heading']} - Step {numbered.group(1)}"
                current_step_lines = [line]
            else:
                current_step_lines.append(line)

        if current_step_lines:
            sub_chunks.append(RawChunk(
                content="\n".join(current_step_lines),
                name=step_name,
                chunk_type="protocol_step",
                index=base_index + len(sub_chunks),
                parent_heading=section["parent_heading"],
                location=f"L{step_start}",
                extra={"step_markers": 1},
            ))
        return sub_chunks
