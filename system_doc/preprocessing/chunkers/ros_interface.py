"""ROS 接口定义分块器 — 处理 .msg/.srv/.action 文件"""

from __future__ import annotations
import re
from pathlib import Path

from .base import BaseChunker, RawChunk


# 匹配字段定义行: type name (可能带默认值和注释)
_FIELD_RE = re.compile(
    r"^([a-zA-Z_][\w/\[\]]*)\s+([a-z_][\w]*)"
    r"(?:\s*=\s*(.+?))?"
    r"(?:\s*#\s*(.+))?$"
)
# 匹配常量定义行: type NAME=value
_CONST_RE = re.compile(
    r"^([a-zA-Z_][\w]*)\s+([A-Z][A-Z0-9_]*)\s*=\s*(.+?)(?:\s*#\s*(.+))?$"
)
# 分隔符（srv 用 ---，action 用 --- 分三段）
_SEPARATOR = re.compile(r"^---\s*$")


class RosInterfaceChunker(BaseChunker):
    """
    处理 ROS2 接口定义文件:
    - .msg: 单段消息字段
    - .srv: Request --- Response 两段
    - .action: Goal --- Result --- Feedback 三段

    每个常量定义和每个字段组（按段）各生成一个 chunk。
    """

    @property
    def supported_doc_type(self) -> str:
        return "ros_interface"

    def chunk(self, content: str, file_path: Path) -> list[RawChunk]:
        suffix = file_path.suffix.lower()
        interface_name = file_path.stem

        # 按 --- 分段
        sections = re.split(r"\n---\s*\n", content)

        if suffix == ".srv":
            section_names = ["Request", "Response"]
        elif suffix == ".action":
            section_names = ["Goal", "Result", "Feedback"]
        else:
            section_names = ["Message"]

        chunks = []
        for i, section in enumerate(sections):
            sec_name = section_names[i] if i < len(section_names) else f"Section{i}"
            # 提取常量
            constants = self._extract_constants(section)
            for const in constants:
                chunks.append(RawChunk(
                    content=const["raw"],
                    name=f"{interface_name}.{const['name']}",
                    chunk_type="constant",
                    index=len(chunks),
                    parent_heading=f"{interface_name}/{sec_name}",
                    location=f"{sec_name}",
                    extra={
                        "name": const["name"],
                        "type": const["type"],
                        "value": const["value"],
                        "description": const.get("comment", ""),
                        "interface": interface_name,
                        "section": sec_name,
                    },
                ))

            # 提取字段组
            fields = self._extract_fields(section)
            if fields:
                field_text = "\n".join(f["raw"] for f in fields)
                field_extra = {
                    "interface": interface_name,
                    "section": sec_name,
                    "fields": [
                        {"name": f["name"], "type": f["type"],
                         "description": f.get("comment", "")}
                        for f in fields
                    ],
                }
                chunks.append(RawChunk(
                    content=field_text,
                    name=f"{interface_name}.{sec_name}",
                    chunk_type="message",
                    index=len(chunks),
                    parent_heading=interface_name,
                    location=f"{sec_name}",
                    extra=field_extra,
                ))

        return chunks

    def _extract_constants(self, section: str) -> list[dict]:
        results = []
        for line in section.splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            m = _CONST_RE.match(line)
            if m:
                results.append({
                    "raw": line,
                    "type": m.group(1),
                    "name": m.group(2),
                    "value": m.group(3).strip(),
                    "comment": m.group(4) or "",
                })
        return results

    def _extract_fields(self, section: str) -> list[dict]:
        results = []
        for line in section.splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            # 跳过常量行
            if _CONST_RE.match(line):
                continue
            m = _FIELD_RE.match(line)
            if m:
                results.append({
                    "raw": line,
                    "type": m.group(1),
                    "name": m.group(2),
                    "default": m.group(3) or "",
                    "comment": m.group(4) or "",
                })
        return results
