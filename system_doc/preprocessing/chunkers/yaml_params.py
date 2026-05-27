"""YAML 参数文件分块器 — 处理 ROS2 参数配置文件"""

from __future__ import annotations
import re
from pathlib import Path
from typing import Any

import yaml

from .base import BaseChunker, RawChunk


class YamlParamChunker(BaseChunker):
    """
    处理 ROS2 YAML 参数文件（如 burger.yaml）。
    将嵌套的参数树展平为独立的 RawChunk，每个叶节点参数一个块。

    示例输入:
        turtlebot3_node:
          ros__parameters:
            wheels:
              separation: 0.160
              radius: 0.033

    输出: 两个 chunk —— wheels.separation=0.160, wheels.radius=0.033
    """

    @property
    def supported_doc_type(self) -> str:
        return "yaml_params"

    def chunk(self, content: str, file_path: Path) -> list[RawChunk]:
        data = yaml.safe_load(content)
        if not isinstance(data, dict):
            return []

        chunks = []
        # ROS2 参数文件通常有 node_name.ros__parameters 结构
        params = self._find_params_root(data)
        self._flatten(params, [], chunks, file_path)
        return chunks

    def _find_params_root(self, data: dict) -> dict:
        """找到参数根节点（跳过 node_name/ros__parameters 包装层）"""
        # 尝试 ROS2 标准结构: {node: {ros__parameters: {...}}}
        for key, val in data.items():
            if isinstance(val, dict):
                if "ros__parameters" in val:
                    return val["ros__parameters"]
                # 递归一层
                for k2, v2 in val.items():
                    if k2 == "ros__parameters" and isinstance(v2, dict):
                        return v2
        # 没有 ros__parameters 包装，直接返回顶层
        return data

    def _flatten(
        self, node: Any, path: list[str], chunks: list[RawChunk], file_path: Path
    ):
        """递归展平参数树，叶节点生成 chunk"""
        if isinstance(node, dict):
            for key, val in node.items():
                self._flatten(val, path + [str(key)], chunks, file_path)
        elif isinstance(node, list):
            # 列表值作为整体保留
            param_path = ".".join(path)
            chunks.append(RawChunk(
                content=f"{param_path}: {node}",
                name=param_path,
                chunk_type=self._infer_type(path, node),
                index=len(chunks),
                parent_heading=path[0] if path else "",
                location=f"$.{param_path}",
                extra={"path": param_path, "value": node, "type": "list"},
            ))
        else:
            # 叶节点（标量值）
            param_path = ".".join(path)
            extra: dict[str, Any] = {
                "path": param_path,
                "name": path[-1] if path else "",
                "value": node,
                "type": type(node).__name__,
            }
            # 尝试推断单位（从参数名）
            unit = self._guess_unit(path[-1] if path else "")
            if unit:
                extra["unit"] = unit

            # 数值参数提取 default
            if isinstance(node, (int, float)):
                extra["default"] = node

            chunks.append(RawChunk(
                content=f"{param_path}: {node}",
                name=param_path,
                chunk_type=self._infer_type(path, node),
                index=len(chunks),
                parent_heading=path[0] if path else "",
                location=f"$.{param_path}",
                extra=extra,
            ))

    def _infer_type(self, path: list[str], value: Any) -> str:
        """从路径和值推断参数类型"""
        path_str = ".".join(path).lower()
        if any(k in path_str for k in ("vel", "speed", "accel", "radius", "separation")):
            return "parameter"
        if any(k in path_str for k in ("topic", "frame", "plugin")):
            return "config"
        if isinstance(value, (int, float)):
            return "parameter"
        if isinstance(value, bool):
            return "config"
        return "config"

    def _guess_unit(self, name: str) -> str:
        """从参数名猜测单位"""
        name_lower = name.lower()
        unit_hints = {
            "vel": "m/s", "speed": "m/s", "accel": "m/s^2",
            "radius": "m", "separation": "m", "distance": "m",
            "theta": "rad/s", "angular": "rad/s", "yaw": "rad",
            "freq": "Hz", "rate": "Hz", "timeout": "s", "time": "s",
        }
        for hint, unit in unit_hints.items():
            if hint in name_lower:
                return unit
        return ""
