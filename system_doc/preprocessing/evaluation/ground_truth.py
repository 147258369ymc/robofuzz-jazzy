"""
独立 Ground Truth 提取器。
不复用流水线的 chunker 代码，确保评估独立性。
通过注册机制支持扩展新的文件格式。
"""

from __future__ import annotations
import json
import re
from pathlib import Path
from xml.etree import ElementTree as ET
from dataclasses import dataclass
from typing import Callable


@dataclass
class GroundTruthEntry:
    """一条 ground truth 记录"""
    name: str
    source_file: str
    entity_type: str  # parameter, message, command, etc.
    extra: dict | None = None  # 可选的额外验证字段


# 提取器注册表
_EXTRACTORS: dict[str, Callable[[Path, dict], list[GroundTruthEntry]]] = {}


def register_extractor(name: str):
    """装饰器：注册一个 ground truth 提取器"""
    def decorator(func):
        _EXTRACTORS[name] = func
        return func
    return decorator


def get_extractor(name: str) -> Callable:
    if name not in _EXTRACTORS:
        raise ValueError(f"Unknown extractor '{name}'. Available: {list(_EXTRACTORS.keys())}")
    return _EXTRACTORS[name]


# === 通用提取器 ===

@register_extractor("json_array")
def extract_json_array(path: Path, params: dict) -> list[GroundTruthEntry]:
    """从 JSON 数组中提取实体"""
    array_key = params.get("array_key")
    name_field = params.get("name_field", "name")
    entity_type = params.get("entity_type", "parameter")

    data = json.loads(path.read_text(encoding="utf-8"))
    if array_key:
        items = data.get(array_key, [])
    elif isinstance(data, list):
        items = data
    else:
        # 自动找最大数组
        items = max((v for v in data.values() if isinstance(v, list)), key=len, default=[])

    entries = []
    for item in items:
        if isinstance(item, dict) and name_field in item:
            entries.append(GroundTruthEntry(
                name=str(item[name_field]),
                source_file=str(path),
                entity_type=entity_type,
                extra=item,
            ))
    return entries


@register_extractor("xml_elements")
def extract_xml_elements(path: Path, params: dict) -> list[GroundTruthEntry]:
    """从 XML 元素中提取实体"""
    tag = params.get("element_tag")
    name_attr = params.get("name_attr", "name")
    name_child = params.get("name_child", "name")
    entity_type = params.get("entity_type", "parameter")

    content = path.read_text(encoding="utf-8")
    root = ET.fromstring(content)

    entries = []
    elements = root.iter(tag) if tag else root
    for elem in elements:
        name = elem.get(name_attr, "") or elem.findtext(name_child, "")
        if name:
            entries.append(GroundTruthEntry(
                name=name,
                source_file=str(path),
                entity_type=entity_type,
            ))
    return entries


@register_extractor("md_table_column")
def extract_md_table_column(path: Path, params: dict) -> list[GroundTruthEntry]:
    """从 Markdown 表格的指定列提取实体名"""
    name_column = params.get("name_column", 0)
    entity_type = params.get("entity_type", "message")
    section_filter = params.get("section_filter", None)  # 只处理指定 section

    content = path.read_text(encoding="utf-8")
    lines = content.splitlines()

    entries = []
    current_section = ""
    header_seen = False
    sep_re = re.compile(r"^\s*\|[\s\-:|]+\|\s*$")

    for line in lines:
        heading = re.match(r"^#{1,6}\s+(.+)$", line)
        if heading:
            current_section = heading.group(1).strip()
            header_seen = False
            continue

        if section_filter and section_filter not in current_section:
            continue

        if sep_re.match(line):
            continue

        row = re.match(r"^\s*\|(.+)\|\s*$", line)
        if row:
            cells = [c.strip() for c in row.group(1).split("|")]
            if not header_seen:
                header_seen = True
                continue  # 跳过表头
            if name_column < len(cells):
                name = cells[name_column]
                if name and not name.startswith("-"):
                    entries.append(GroundTruthEntry(
                        name=name,
                        source_file=str(path),
                        entity_type=entity_type,
                    ))
    return entries


@register_extractor("md_headings")
def extract_md_headings(path: Path, params: dict) -> list[GroundTruthEntry]:
    """从 Markdown heading 提取实体"""
    min_level = params.get("min_level", 2)
    max_level = params.get("max_level", 3)
    entity_type = params.get("entity_type", "message")
    pattern = params.get("name_pattern", None)  # 可选正则过滤
    name_cleanup = params.get("name_cleanup", True)  # 清理名称中的附加信息

    content = path.read_text(encoding="utf-8")
    entries = []

    for m in re.finditer(r"^(#{1,6})\s+(.+)$", content, re.MULTILINE):
        level = len(m.group(1))
        title = m.group(2).strip()
        if min_level <= level <= max_level:
            if pattern and not re.search(pattern, title):
                continue
            # 清理名称：去掉 "(ID: xx)" 等后缀
            name = title
            if name_cleanup:
                name = re.sub(r"\s*\(.*?\)\s*$", "", name).strip()
                name = re.sub(r"\s*\[.*?\]\s*$", "", name).strip()
            entries.append(GroundTruthEntry(
                name=name,
                source_file=str(path),
                entity_type=entity_type,
            ))
    return entries


# === Ground Truth 配置 ===

@dataclass
class GroundTruthConfig:
    """单个源文件的 ground truth 提取配置"""
    file_path: str          # 相对于 doc_root
    extractor: str          # 提取器名称
    params: dict            # 提取器参数


# PX4 预定义配置
PX4_GROUND_TRUTH_CONFIGS = [
    GroundTruthConfig(
        file_path="parameters.json",
        extractor="json_array",
        params={"array_key": "parameters", "name_field": "name", "entity_type": "parameter"},
    ),
    GroundTruthConfig(
        file_path="parameters.xml",
        extractor="xml_elements",
        params={"element_tag": "parameter", "entity_type": "parameter"},
    ),
    GroundTruthConfig(
        file_path="uorb_message_reference.md",
        extractor="md_table_column",
        params={"name_column": 0, "entity_type": "message"},
    ),
    GroundTruthConfig(
        file_path="vehicle_command_reference.md",
        extractor="md_table_column",
        params={"name_column": 0, "entity_type": "command", "section_filter": "Command"},
    ),
    GroundTruthConfig(
        file_path="mavlink_common_messages.md",
        extractor="md_headings",
        params={"min_level": 2, "max_level": 2, "entity_type": "message",
                "name_pattern": r"\(ID:\s*\d+\)"},
    ),
]


def get_ground_truth_configs(target: str) -> list[GroundTruthConfig]:
    configs = {
        "px4": PX4_GROUND_TRUTH_CONFIGS,
    }
    return configs.get(target, [])
