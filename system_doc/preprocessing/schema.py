"""SpecBlock schema 定义 — 预处理流水线的统一中间表示"""

from __future__ import annotations
from dataclasses import dataclass, field, asdict
from typing import Any
import json


@dataclass
class Provenance:
    """来源追溯信息"""
    source_file: str          # 相对于 system_doc/ 的路径
    source_system: str        # "px4" | "turtlebot3" | "ardupilot" ...
    version: str              # commit hash 或版本号
    location: str             # 行号/section/xpath/json_path
    doc_type: str             # structured_data | tabular_markdown | prose_markdown | protocol_spec

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class ChunkMeta:
    """分块时产生的元数据"""
    chunk_index: int          # 在原文档中的顺序
    parent_heading: str = ""  # 所属的上级 heading（Markdown 文档）
    sibling_count: int = 0    # 同级 chunk 数量（用于上下文判断）


@dataclass
class SpecBlock:
    """预处理流水线的统一输出单元"""
    # 身份标识
    block_id: str             # 全局唯一，如 "px4.param.MPC_VEL_MANUAL"
    block_type: str           # parameter | message | field | command | protocol_step | hardware_spec | config_desc

    # 内容
    name: str                 # 实体名称
    raw_content: str          # 原始文本片段
    structured_fields: dict[str, Any] = field(default_factory=dict)
    natural_language: str = ""  # 自然语言描述

    # 追溯
    provenance: Provenance | None = None
    chunk_meta: ChunkMeta | None = None

    # 关系提示（本块中提到的其他实体名称）
    references: list[str] = field(default_factory=list)

    # 语义标签（由下游 LLM 或规则生成）
    tags: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        d = asdict(self)
        return d

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, indent=indent)

    @classmethod
    def from_dict(cls, data: dict) -> "SpecBlock":
        prov = data.pop("provenance", None)
        meta = data.pop("chunk_meta", None)
        if prov and isinstance(prov, dict):
            prov = Provenance(**prov)
        if meta and isinstance(meta, dict):
            meta = ChunkMeta(**meta)
        return cls(**data, provenance=prov, chunk_meta=meta)
