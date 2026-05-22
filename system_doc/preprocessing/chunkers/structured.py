"""结构化数据分块器 — 处理 JSON/XML 格式的规约文件"""

from __future__ import annotations
import json
import re
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

from .base import BaseChunker, RawChunk


class StructuredDataChunker(BaseChunker):
    """
    处理 JSON/XML 结构化数据。
    - JSON: 按顶层数组元素或指定 key 下的数组元素分块
    - XML: 按指定标签或第二层子元素分块
    """

    def __init__(self, json_array_key: str | None = None, xml_element_tag: str | None = None):
        """
        Args:
            json_array_key: JSON 中包含主数据的数组字段名（如 "parameters"）。
                           None 则自动探测。
            xml_element_tag: XML 中要分块的元素标签（如 "parameter"）。
                           None 则自动探测。
        """
        self.json_array_key = json_array_key
        self.xml_element_tag = xml_element_tag

    @property
    def supported_doc_type(self) -> str:
        return "structured_data"

    def chunk(self, content: str, file_path: Path) -> list[RawChunk]:
        suffix = file_path.suffix.lower()
        if suffix == ".json":
            return self._chunk_json(content, file_path)
        elif suffix == ".xml":
            return self._chunk_xml(content, file_path)
        return []

    def _chunk_xml(self, content: str, file_path: Path) -> list[RawChunk]:
        root = ET.fromstring(content)
        target_tag = self.xml_element_tag
        chunks = []

        if target_tag:
            elements = root.iter(target_tag)
        else:
            # 自动探测：找出现次数最多的第二层标签
            tag_counts: dict[str, int] = {}
            for child in root:
                for sub in child:
                    tag_counts[sub.tag] = tag_counts.get(sub.tag, 0) + 1
            if not tag_counts:
                # 退回到直接子元素
                elements = list(root)
                target_tag = root.tag
            else:
                target_tag = max(tag_counts, key=tag_counts.get)
                elements = root.iter(target_tag)
            self.xml_element_tag = target_tag

        for i, elem in enumerate(elements):
            name = elem.get("name", "") or elem.findtext("name", "")
            if not name:
                name = f"{target_tag}_{i}"
            raw_text = ET.tostring(elem, encoding="unicode", method="xml")
            # 提取所有属性和子文本为 extra
            extra = dict(elem.attrib)
            for child in elem:
                if child.text and child.text.strip():
                    extra[child.tag] = child.text.strip()
            chunks.append(RawChunk(
                content=raw_text,
                name=name,
                chunk_type=self._infer_xml_type(elem),
                index=i,
                location=f"//{target_tag}[{i}]",
                extra=extra,
            ))
        return chunks

    def _infer_xml_type(self, elem: ET.Element) -> str:
        tag = elem.tag.lower()
        if "param" in tag:
            return "parameter"
        if "msg" in tag or "message" in tag:
            return "message"
        if "cmd" in tag or "command" in tag:
            return "command"
        return "data_entry"

    def _chunk_json(self, content: str, file_path: Path) -> list[RawChunk]:
        data = json.loads(content)
        items = self._find_json_array(data)
        chunks = []
        for i, item in enumerate(items):
            name = self._extract_name(item)
            chunk_type = self._infer_chunk_type(item)
            chunks.append(RawChunk(
                content=json.dumps(item, ensure_ascii=False, indent=2),
                name=name,
                chunk_type=chunk_type,
                index=i,
                location=f"$.{self.json_array_key or 'root'}[{i}]",
                extra=item if isinstance(item, dict) else {"value": item},
            ))
        return chunks

    def _find_json_array(self, data: Any) -> list:
        """自动探测 JSON 中的主数组"""
        if self.json_array_key and isinstance(data, dict):
            return data.get(self.json_array_key, [])
        if isinstance(data, list):
            return data
        if isinstance(data, dict):
            # 找最大的数组字段
            best_key, best_len = None, 0
            for k, v in data.items():
                if isinstance(v, list) and len(v) > best_len:
                    best_key, best_len = k, len(v)
            if best_key:
                self.json_array_key = best_key
                return data[best_key]
        return []

    def _extract_name(self, item: Any) -> str:
        if isinstance(item, dict):
            for key in ("name", "id", "param_id", "msg_name", "title"):
                if key in item:
                    return str(item[key])
        return str(item)[:50]

    def _infer_chunk_type(self, item: Any) -> str:
        if not isinstance(item, dict):
            return "data_entry"
        keys = set(item.keys())
        if {"min", "max", "default"} & keys:
            return "parameter"
        if {"fields", "field"} & keys:
            return "message"
        if {"command", "cmd"} & keys:
            return "command"
        return "data_entry"
