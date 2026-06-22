"""Agent 工具定义 — 供 tool_use Agent 调用的能力集合"""

from __future__ import annotations
import json
import re
from pathlib import Path
from typing import Any

import yaml as yaml_lib


# ============================================================
# Tool Schema 定义（Anthropic tool_use 格式）
# ============================================================

TOOL_DEFINITIONS = [
    {
        "name": "query_index",
        "description": (
            "查询 SpecIndex。可按 tag、entity 名称、block_type 搜索。"
            "返回匹配的 block_id 列表（最多 30 条）。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "by_tag": {
                    "type": "string",
                    "description": "按语义标签查询，如 velocity_constraint",
                },
                "by_entity": {
                    "type": "string",
                    "description": "按实体名查询，如 VehicleLocalPosition",
                },
                "by_type": {
                    "type": "string",
                    "description": "按 block_type 过滤，如 parameter/message/field",
                },
            },
        },
    },
    {
        "name": "read_block",
        "description": (
            "读取一个 SpecBlock 的完整内容。"
            "输入 block_id，返回该 block 的 JSON 数据。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "block_id": {
                    "type": "string",
                    "description": "要读取的 block_id",
                },
            },
            "required": ["block_id"],
        },
    },
    {
        "name": "list_specs",
        "description": "列出已有的 OracleIR spec 文件名，用于了解已覆盖哪些参数。",
        "input_schema": {
            "type": "object",
            "properties": {},
        },
    },
    {
        "name": "read_spec",
        "description": (
            "读取一个已有的 OracleIR YAML spec 文件内容，用作参考范例。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "filename": {
                    "type": "string",
                    "description": "spec 文件名（不含路径），如 03_velocity_xy.yaml",
                },
            },
            "required": ["filename"],
        },
    },
    {
        "name": "validate_yaml",
        "description": (
            "校验生成的 OracleIR YAML 是否符合 schema。"
            "返回校验结果：通过/失败 + 错误列表。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "yaml_content": {
                    "type": "string",
                    "description": "要校验的 YAML 文本",
                },
            },
            "required": ["yaml_content"],
        },
    },
    {
        "name": "save_spec",
        "description": (
            "将最终通过校验的 OracleIR YAML 保存到 specs 目录。"
            "只有校验通过后才应调用此工具。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "filename": {
                    "type": "string",
                    "description": "保存的文件名（不含路径），如 px4_range_mpc_xy_vel_max.yaml",
                },
                "yaml_content": {
                    "type": "string",
                    "description": "要保存的 YAML 文本",
                },
            },
            "required": ["filename", "yaml_content"],
        },
    },
]


# ============================================================
# Tool 执行器 — 处理 Agent 的 tool_use 请求
# ============================================================

class ToolExecutor:
    """执行 Agent 发出的工具调用，返回结果"""

    def __init__(self, blocks_dir: Path, index_path: Path, specs_dir: Path):
        self.blocks_dir = blocks_dir
        self.specs_dir = specs_dir
        self.specs_dir.mkdir(parents=True, exist_ok=True)

        # 加载索引
        self.index = json.loads(index_path.read_text(encoding="utf-8"))

        # 懒加载 blocks 缓存
        self._blocks_cache: dict[str, dict] | None = None

    @property
    def blocks(self) -> dict[str, dict]:
        if self._blocks_cache is None:
            self._blocks_cache = {}
            for f in self.blocks_dir.glob("*.json"):
                data = json.loads(f.read_text(encoding="utf-8"))
                self._blocks_cache[data["block_id"]] = data
        return self._blocks_cache

    def execute(self, tool_name: str, tool_input: dict) -> str:
        """分发工具调用，返回结果字符串"""
        handler = getattr(self, f"_handle_{tool_name}", None)
        if handler is None:
            return json.dumps({"error": f"未知工具: {tool_name}"})
        try:
            result = handler(tool_input)
            return result if isinstance(result, str) else json.dumps(
                result, ensure_ascii=False, indent=2
            )
        except Exception as e:
            return json.dumps({"error": str(e)})

    def _handle_query_index(self, inp: dict) -> Any:
        results: list[str] = []
        tag = inp.get("by_tag")
        entity = inp.get("by_entity")
        block_type = inp.get("by_type")

        if tag:
            tag_index = self.index.get("tag_index", {})
            results.extend(tag_index.get(tag, []))
        if entity:
            entity_index = self.index.get("entity_index", {})
            results.extend(entity_index.get(entity, []))

        # 去重
        seen = set()
        unique = []
        for r in results:
            if r not in seen:
                seen.add(r)
                unique.append(r)

        # 按 block_type 过滤
        if block_type:
            unique = [
                bid for bid in unique
                if self.blocks.get(bid, {}).get("block_type") == block_type
            ]

        return {"block_ids": unique[:30], "total": len(unique)}

    def _handle_read_block(self, inp: dict) -> Any:
        block_id = inp["block_id"]
        block = self.blocks.get(block_id)
        if block is None:
            # 尝试从文件加载
            block_path = self.blocks_dir / f"{block_id}.json"
            if block_path.exists():
                block = json.loads(block_path.read_text(encoding="utf-8"))
            else:
                return {"error": f"Block not found: {block_id}"}
        return block

    def _handle_list_specs(self, inp: dict) -> Any:
        files = sorted(f.name for f in self.specs_dir.glob("*.yaml"))
        return {"specs": files, "count": len(files)}

    def _handle_read_spec(self, inp: dict) -> Any:
        filename = inp["filename"]
        path = self.specs_dir / filename
        if not path.exists():
            return {"error": f"Spec not found: {filename}"}
        return {"filename": filename, "content": path.read_text(encoding="utf-8")}

    def _handle_validate_yaml(self, inp: dict) -> Any:
        yaml_content = inp["yaml_content"]
        # 剥离 LLM 可能添加的 markdown 代码围栏
        yaml_content = re.sub(r'^```(?:yaml|yml)?\s*\n', '', yaml_content.strip())
        yaml_content = re.sub(r'\n```\s*$', '', yaml_content)
        try:
            data = yaml_lib.safe_load(yaml_content)
        except yaml_lib.YAMLError as e:
            return {"valid": False, "errors": [f"YAML 语法错误: {e}"]}

        if not isinstance(data, dict):
            return {"valid": False, "errors": ["输出不是有效的 YAML 字典"]}

        try:
            from src.oracle_ir.transform.parser import _dict_to_oracle_ir
            from src.oracle_ir.transform.validator import validate_oracle_ir
            ir = _dict_to_oracle_ir(data)
            result = validate_oracle_ir(ir)
            return {"valid": result.valid, "errors": result.errors}
        except Exception as e:
            return {"valid": False, "errors": [f"解析失败: {e}"]}

    def _handle_save_spec(self, inp: dict) -> Any:
        filename = inp["filename"]
        yaml_content = inp["yaml_content"]

        # 剥离可能的 markdown 围栏
        yaml_content = re.sub(r'^```(?:yaml|yml)?\s*\n', '', yaml_content.strip())
        yaml_content = re.sub(r'\n```\s*$', '', yaml_content)

        # 安全检查：文件名不能包含路径分隔符
        if "/" in filename or "\\" in filename:
            return {"error": "文件名不能包含路径分隔符"}

        path = self.specs_dir / filename
        path.write_text(yaml_content, encoding="utf-8")
        return {"saved": str(path), "size": len(yaml_content)}
