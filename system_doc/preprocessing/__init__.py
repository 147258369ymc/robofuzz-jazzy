"""
多源异构规约预处理流水线 (Multi-Source Specification Preprocessing Pipeline)

将异构格式的规约文档（JSON/XML/Markdown/协议描述）统一转换为结构化 SpecBlock，
支持跨源索引构建和来源追溯，为下游 oracle 提取 agent 提供高质量输入。
"""

from .schema import SpecBlock, Provenance, ChunkMeta
from .pipeline import PreprocessingPipeline
from .detector import DocTypeDetector

__all__ = [
    "SpecBlock",
    "Provenance",
    "ChunkMeta",
    "PreprocessingPipeline",
    "DocTypeDetector",
]
