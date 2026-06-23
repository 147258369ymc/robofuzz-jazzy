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
                extra={
                    "path": param_path,
                    "value": node,
                    "type": "list",
                    **self._semantic_fields(path, node, file_path),
                },
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
            extra.update(self._semantic_fields(path, node, file_path))
            # 尝试推断单位（从参数名）
            unit = extra.get("unit") or self._guess_unit(path[-1] if path else "")
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

    def _semantic_fields(self, path: list[str], value: Any, file_path: Path) -> dict[str, Any]:
        """Infer reusable semantic hints for downstream OracleIR generation."""
        leaf = path[-1] if path else ""
        path_str = ".".join(path)
        fields: dict[str, Any] = {
            "source_role": self._source_role(file_path),
            "param_role": self._param_role(leaf, value),
            "semantic_tags": [],
            "preferred_for_oracle_generation": False,
        }

        bound_kind = self._bound_kind(leaf)
        if bound_kind:
            fields["bound_kind"] = bound_kind

        quantity = self._quantity(path)
        if quantity:
            fields["quantity"] = quantity

        joint = self._joint_name(path)
        if joint:
            fields["joint"] = joint
            unit = self._joint_unit(quantity, joint)
            if unit:
                fields["unit"] = unit

        tags = set()
        role = fields["param_role"]
        if role == "enable_flag":
            tags.add("limit_enable_flag")
            if quantity:
                tags.add(f"{quantity}_enable_flag")
        elif role in {"numeric_upper_bound", "numeric_lower_bound"}:
            tags.add("numeric_bound")
            if quantity:
                tags.add(f"{quantity}_bound")
            if self._is_preferred_oracle_candidate(fields):
                fields["preferred_for_oracle_generation"] = True
                fields["oracle_type_hint"] = "range_bound"
                tags.add("oracle_numeric_bound_candidate")
                if quantity:
                    tags.add(f"oracle_{quantity}_bound_candidate")

        if path_str.startswith("joint_limits."):
            tags.add("joint_limit_config")

        fields["semantic_tags"] = sorted(tags)
        return fields

    def _source_role(self, file_path: Path) -> str:
        name = file_path.name
        if name == "hard_joint_limits.yaml":
            return "hard_joint_limits"
        if name == "joint_limits.yaml":
            return "default_joint_limits"
        if name == "initial_positions.yaml":
            return "initial_positions"
        return "yaml_params"

    def _param_role(self, leaf: str, value: Any) -> str:
        leaf_lower = leaf.lower()
        if isinstance(value, bool) or leaf_lower.startswith("has_") or leaf_lower.startswith("enable_"):
            return "enable_flag"
        if self._bound_kind(leaf) == "max":
            return "numeric_upper_bound"
        if self._bound_kind(leaf) == "min":
            return "numeric_lower_bound"
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            return "numeric_value"
        return "config_value"

    def _bound_kind(self, leaf: str) -> str:
        leaf_lower = leaf.lower()
        if leaf_lower.startswith(("max_", "upper", "soft_upper")):
            return "max"
        if leaf_lower.startswith(("min_", "lower", "soft_lower")):
            return "min"
        return ""

    def _quantity(self, path: list[str]) -> str:
        text = ".".join(path).lower()
        if "joint_limits." in text:
            if "velocity" in text:
                return "joint_velocity"
            if "acceleration" in text or "accel" in text:
                return "joint_acceleration"
            if "jerk" in text:
                return "joint_jerk"
            if any(key in text for key in ("position", "lower", "upper")):
                return "joint_position"
        if "velocity" in text or "speed" in text:
            return "velocity"
        if "acceleration" in text or "accel" in text:
            return "acceleration"
        return ""

    def _joint_name(self, path: list[str]) -> str:
        if len(path) >= 3 and path[0] == "joint_limits":
            return path[1]
        if len(path) >= 2 and path[0] == "initial_positions":
            return path[1]
        return ""

    def _joint_unit(self, quantity: str, joint: str) -> str:
        is_prismatic_like = "finger" in joint
        if quantity == "joint_velocity":
            return "m/s" if is_prismatic_like else "rad/s"
        if quantity == "joint_acceleration":
            return "m/s^2" if is_prismatic_like else "rad/s^2"
        if quantity == "joint_jerk":
            return "m/s^3" if is_prismatic_like else "rad/s^3"
        if quantity == "joint_position":
            return "m" if is_prismatic_like else "rad"
        return ""

    def _is_preferred_oracle_candidate(self, fields: dict[str, Any]) -> bool:
        return (
            fields.get("source_role") == "default_joint_limits"
            and fields.get("param_role") in {"numeric_upper_bound", "numeric_lower_bound"}
            and fields.get("quantity") in {"joint_velocity", "joint_acceleration", "joint_jerk"}
        )

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
