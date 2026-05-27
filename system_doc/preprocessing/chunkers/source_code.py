"""源码约束提取分块器 — 从 C++/Python 源码中提取常量和约束定义"""

from __future__ import annotations
import re
from pathlib import Path

from .base import BaseChunker, RawChunk


# C++ 常量模式
_CPP_DEFINE = re.compile(
    r"^\s*#define\s+([A-Z_][A-Z0-9_]*)\s+([0-9.eE+\-]+)"
    r"(?:\s*(?://|/\*)\s*(.+?)(?:\*/)?)?$",
    re.MULTILINE,
)
_CPP_CONST = re.compile(
    r"(?:const|constexpr|static)\s+\w+\s+([A-Z_][A-Z0-9_a-z]*)\s*=\s*([0-9.eE+\-]+)"
    r"(?:\s*;\s*(?://|/\*)\s*(.+?)(?:\*/)?)?",
)
# 成员赋值: obj->field = value 或 obj.field = value
_CPP_MEMBER_ASSIGN = re.compile(
    r"(\w+(?:->|\.))([a-z_][a-z0-9_]*)\s*=\s*([0-9.eE+\-]+)\s*;"
    r"(?:\s*(?://|/\*)\s*(.+?)(?:\*/)?)?",
)

# Python 常量模式
_PY_CONST = re.compile(
    r"^([A-Z_][A-Z0-9_]*)\s*=\s*([0-9.eE+\-]+)"
    r"(?:\s*#\s*(.+))?$",
    re.MULTILINE,
)

# C++ 控制表/结构体字段（OpenCR 风格）
_CPP_STRUCT_FIELD = re.compile(
    r'\{\s*"([^"]+)"\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\w+)',
)


class SourceCodeChunker(BaseChunker):
    """
    从 C++/Python 源码中提取安全相关的常量和约束。

    提取目标:
    - #define MACRO value
    - const/constexpr TYPE NAME = value
    - Python: UPPER_CASE = numeric_value
    - 结构体初始化中的数值字段

    不提取: 函数体逻辑、类定义、import 等（那些对 oracle 无用）
    """

    @property
    def supported_doc_type(self) -> str:
        return "source_code"

    def chunk(self, content: str, file_path: Path) -> list[RawChunk]:
        suffix = file_path.suffix.lower()
        if suffix == ".py":
            return self._chunk_python(content, file_path)
        else:
            return self._chunk_cpp(content, file_path)

    def _chunk_python(self, content: str, file_path: Path) -> list[RawChunk]:
        chunks = []
        for m in _PY_CONST.finditer(content):
            name = m.group(1)
            value = m.group(2)
            comment = m.group(3) or ""
            chunks.append(RawChunk(
                content=m.group(0),
                name=name,
                chunk_type="parameter",
                index=len(chunks),
                parent_heading=file_path.stem,
                location=f"L{content[:m.start()].count(chr(10)) + 1}",
                extra={
                    "name": name,
                    "default": self._parse_num(value),
                    "value": value,
                    "description": comment.strip(),
                    "source": "python_const",
                },
            ))
        return chunks

    def _chunk_cpp(self, content: str, file_path: Path) -> list[RawChunk]:
        chunks = []

        # #define 宏
        for m in _CPP_DEFINE.finditer(content):
            chunks.append(self._make_chunk(
                m.group(0), m.group(1), m.group(2), m.group(3),
                "cpp_define", content, m.start(), file_path,
            ))

        # const/constexpr
        for m in _CPP_CONST.finditer(content):
            chunks.append(self._make_chunk(
                m.group(0), m.group(1), m.group(2), m.group(3),
                "cpp_const", content, m.start(), file_path,
            ))

        # 成员赋值 (scan->range_min = 0.12)
        for m in _CPP_MEMBER_ASSIGN.finditer(content):
            field_name = m.group(2)
            value = m.group(3)
            comment = m.group(4)
            chunks.append(self._make_chunk(
                m.group(0), field_name, value, comment,
                "cpp_member_assign", content, m.start(), file_path,
            ))

        # 结构体字段（控制表风格）
        for m in _CPP_STRUCT_FIELD.finditer(content):
            name = m.group(1)
            addr = m.group(2)
            size = m.group(3)
            access = m.group(4)
            chunks.append(RawChunk(
                content=m.group(0),
                name=name,
                chunk_type="field",
                index=len(chunks),
                parent_heading=file_path.stem,
                location=f"L{content[:m.start()].count(chr(10)) + 1}",
                extra={
                    "name": name,
                    "address": int(addr),
                    "size": int(size),
                    "access": access,
                    "source": "control_table",
                },
            ))

        return chunks

    def _make_chunk(
        self, raw: str, name: str, value: str, comment: str | None,
        source: str, content: str, pos: int, file_path: Path,
    ) -> RawChunk:
        return RawChunk(
            content=raw.strip(),
            name=name,
            chunk_type="parameter",
            index=0,  # will be reassigned
            parent_heading=file_path.stem,
            location=f"L{content[:pos].count(chr(10)) + 1}",
            extra={
                "name": name,
                "default": self._parse_num(value),
                "value": value,
                "description": (comment or "").strip(),
                "source": source,
            },
        )

    def _parse_num(self, s: str) -> float | int | None:
        try:
            if "." in s or "e" in s.lower():
                return float(s)
            return int(s)
        except (ValueError, TypeError):
            return None

