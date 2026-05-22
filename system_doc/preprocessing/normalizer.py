"""归一化器 — 将 RawChunk 转换为统一的 SpecBlock"""

from __future__ import annotations
import re
from pathlib import Path
from typing import Any

from .schema import SpecBlock, Provenance, ChunkMeta
from .chunkers.base import RawChunk


# 常见实体名称模式（用于提取引用关系）
_ENTITY_PATTERNS = [
    re.compile(r"\b([A-Z][a-z]+(?:[A-Z][a-z]+)+)\b"),       # CamelCase: VehicleLocalPosition
    re.compile(r"\b([A-Z][A-Z0-9_]{3,})\b"),                 # UPPER_CASE: MPC_VEL_MANUAL (至少4字符)
    re.compile(r"\b((?:VEHICLE_CMD|MAV_CMD)_[A-Z_]+)\b"),    # 命令名
]

# 引用过滤集：类型名、通用关键词等不应被当作实体引用
_REFERENCE_FILTER = {
    # 数据类型
    "FLOAT", "DOUBLE", "INT32", "UINT8", "UINT16", "UINT32", "UINT64",
    "INT8", "INT16", "BOOL", "STRING", "CHAR", "VOID",
    "Float", "Double", "Int32", "String", "Bool", "Float32", "Float64",
    # 通用关键词
    "ONLY", "READ", "WRITE", "THIS", "THAT", "THEN", "WHEN", "WITH",
    "FROM", "INTO", "OVER", "UNDER", "AFTER", "BEFORE", "BETWEEN",
    "TRUE", "FALSE", "NULL", "NONE", "TODO", "NOTE", "DEPRECATED",
    "IMPORTANT", "WARNING", "REQUIRED", "OPTIONAL", "DEFAULT",
    "ENABLED", "DISABLED", "UNKNOWN", "INVALID", "VALID",
    "MAX", "MIN", "ALL", "ANY", "SET", "GET", "NEW", "OLD",
    # 通用缩写
    "GNSS", "ASCII", "JSON", "XML", "YAML", "GPIO", "SPI", "UART", "USB",
    "NaN", "Inf", "Standard", "Category",
}


class Normalizer:
    """
    将 RawChunk 归一化为 SpecBlock。
    可通过 register_type_handler() 为特定 chunk_type 注册自定义处理逻辑。
    """

    def __init__(self, source_system: str, version: str, base_path: str = ""):
        self.source_system = source_system
        self.version = version
        self.base_path = base_path
        self._handlers: dict[str, callable] = {}

    def register_type_handler(self, chunk_type: str, handler: callable):
        """注册自定义归一化处理器: handler(chunk, normalizer) -> dict of overrides"""
        self._handlers[chunk_type] = handler

    def normalize(self, chunk: RawChunk, file_path: Path, doc_type: str) -> SpecBlock:
        """将 RawChunk 转换为 SpecBlock"""
        rel_path = str(file_path)
        if self.base_path:
            try:
                rel_path = str(file_path.relative_to(self.base_path))
            except ValueError:
                pass

        # 构建 block_id
        block_id = self._build_id(chunk)

        # 提取结构化字段
        structured = self._extract_structured(chunk)

        # 提取自然语言描述
        nl_desc = self._extract_description(chunk)

        # 提取引用关系
        references = self._extract_references(chunk)

        # 应用自定义处理器
        overrides = {}
        if chunk.chunk_type in self._handlers:
            overrides = self._handlers[chunk.chunk_type](chunk, self) or {}

        block = SpecBlock(
            block_id=overrides.get("block_id", block_id),
            block_type=overrides.get("block_type", chunk.chunk_type),
            name=overrides.get("name", chunk.name),
            raw_content=chunk.content,
            structured_fields=overrides.get("structured_fields", structured),
            natural_language=overrides.get("natural_language", nl_desc),
            provenance=Provenance(
                source_file=rel_path,
                source_system=self.source_system,
                version=self.version,
                location=chunk.location,
                doc_type=doc_type,
            ),
            chunk_meta=ChunkMeta(
                chunk_index=chunk.index,
                parent_heading=chunk.parent_heading,
            ),
            references=overrides.get("references", references),
        )
        return block

    def _build_id(self, chunk: RawChunk) -> str:
        safe_name = re.sub(r"[^a-zA-Z0-9_]", "_", chunk.name)[:60]
        return f"{self.source_system}.{chunk.chunk_type}.{safe_name}"

    def _extract_structured(self, chunk: RawChunk) -> dict[str, Any]:
        if not chunk.extra:
            return {}
        # 过滤掉过长的值（保留结构化字段）
        result = {}
        for k, v in chunk.extra.items():
            if isinstance(v, str) and len(v) > 500:
                continue
            result[k] = v
        return result

    def _extract_description(self, chunk: RawChunk) -> str:
        if chunk.extra:
            for key in ("longDesc", "shortDesc", "description", "Description", "desc"):
                if key in chunk.extra and chunk.extra[key]:
                    return str(chunk.extra[key])
        # 从 raw_content 中提取非表格、非代码的文本
        lines = chunk.content.splitlines()
        prose = [l for l in lines if l.strip() and not l.strip().startswith("|")
                 and not l.strip().startswith("#") and not l.strip().startswith("```")]
        return " ".join(prose[:5])[:300]

    def _extract_references(self, chunk: RawChunk) -> list[str]:
        refs = set()
        text = chunk.content
        if chunk.extra:
            text += " " + " ".join(str(v) for v in chunk.extra.values() if isinstance(v, str))
        for pattern in _ENTITY_PATTERNS:
            for match in pattern.finditer(text):
                entity = match.group(1)
                # 过滤掉太短或太通用的
                if len(entity) > 3 and entity != chunk.name and entity not in _REFERENCE_FILTER:
                    refs.add(entity)
        return sorted(refs)[:20]  # 限制数量
