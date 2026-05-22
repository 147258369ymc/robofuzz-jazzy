"""分块器抽象基类"""

from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class RawChunk:
    """分块器输出的原始块"""
    content: str              # 原始文本
    name: str                 # 实体名称（如参数名、消息名）
    chunk_type: str           # 块类型提示
    index: int                # 在文档中的顺序
    parent_heading: str = ""  # 所属上级 heading
    location: str = ""        # 定位信息（行号/path）
    extra: dict[str, Any] | None = None  # 额外结构化信息


class BaseChunker(ABC):
    """
    分块器基类。子类实现 chunk() 方法，将文档内容切分为 RawChunk 列表。
    可迁移性：新目标只需实现对应的 Chunker 子类。
    """

    @abstractmethod
    def chunk(self, content: str, file_path: Path) -> list[RawChunk]:
        """将文档内容切分为 RawChunk 列表"""
        ...

    @property
    @abstractmethod
    def supported_doc_type(self) -> str:
        """返回此分块器支持的 DocType 值"""
        ...
