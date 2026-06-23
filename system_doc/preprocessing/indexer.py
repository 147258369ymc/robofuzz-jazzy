"""跨源索引构建器 — 构建实体索引、类型索引和语义标签索引"""

from __future__ import annotations
import json
import re
from dataclasses import dataclass, field, asdict
from pathlib import Path

from .schema import SpecBlock


@dataclass
class SpecIndex:
    """跨源规约索引"""
    # 按实体名称索引（同一实体可能出现在多个文档中）
    entity_index: dict[str, list[str]] = field(default_factory=dict)
    # 按类型索引
    type_index: dict[str, list[str]] = field(default_factory=dict)
    # 按语义标签索引
    tag_index: dict[str, list[str]] = field(default_factory=dict)
    # 按来源系统索引
    system_index: dict[str, list[str]] = field(default_factory=dict)
    # 跨源引用关系（entity_name → 引用它的 block_ids）
    reference_graph: dict[str, list[str]] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return asdict(self)

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, indent=indent)

    def save(self, path: Path):
        path.write_text(self.to_json(), encoding="utf-8")

    @classmethod
    def load(cls, path: Path) -> "SpecIndex":
        data = json.loads(path.read_text(encoding="utf-8"))
        return cls(**data)

    def query_entity(self, name: str) -> list[str]:
        """查询实体名称对应的所有 block_id"""
        results = set()
        # 精确匹配
        if name in self.entity_index:
            results.update(self.entity_index[name])
        # 模糊匹配（大小写不敏感）
        name_lower = name.lower()
        for key, ids in self.entity_index.items():
            if key.lower() == name_lower:
                results.update(ids)
        return sorted(results)

    def query_type(self, block_type: str) -> list[str]:
        return self.type_index.get(block_type, [])

    def query_tag(self, tag: str) -> list[str]:
        return self.tag_index.get(tag, [])

    def query_references_to(self, entity_name: str) -> list[str]:
        """查询引用了某实体的所有 block_id"""
        return self.reference_graph.get(entity_name, [])


# 通用标签规则（不绑定任何特定目标系统）
_GENERIC_TAG_RULES: list[tuple[re.Pattern, str]] = [
    (re.compile(r"(velocity|vel_max|vel_min|speed|airspd)", re.I), "velocity_constraint"),
    (re.compile(r"(roll|pitch|yaw|tilt|attitude|orientation)", re.I), "attitude_constraint"),
    (re.compile(r"(altitude|height|alt_max|alt_min|climb_rate|sink_rate)", re.I), "altitude_constraint"),
    (re.compile(r"(position\s*(max|min|limit|error)|waypoint|pos_max|pos_min|gps\s*(loss|fail))", re.I), "position_constraint"),
    (re.compile(r"(^|_)(imu|gyro|accel|baro|mag|lidar|sonar|encoder)(_|$)", re.I), "sensor"),
    (re.compile(r"(timeout|timer|delay|interval|deadband)", re.I), "temporal"),
    (re.compile(r"(battery|voltage|current|power|energy)", re.I), "power"),
    (re.compile(r"(motor|actuator|servo|pwm|thrust|torque|effort)", re.I), "actuator"),
    (re.compile(r"(failsafe|fail_act|emergency|abort|estop|e_stop)", re.I), "safety"),
    (re.compile(r"(joint|dof|link|axis)", re.I), "joint_constraint"),
    (re.compile(r"(force|contact|collision|impact)", re.I), "force_constraint"),
    (re.compile(r"(workspace|reach|boundary|limit|envelope)", re.I), "workspace_constraint"),
]


class IndexBuilder:
    """从 SpecBlock 列表构建 SpecIndex"""

    def __init__(self, tag_rules: list[tuple[re.Pattern, str]] | None = None):
        """
        Args:
            tag_rules: 自定义标签规则列表 [(compiled_regex, tag_name), ...]。
                       为 None 时使用通用规则。
                       可通过 TargetDescriptor.get_tag_rules() 获取目标特定规则。
        """
        self._tag_rules = tag_rules if tag_rules is not None else _GENERIC_TAG_RULES

    @classmethod
    def from_descriptor(cls, descriptor_path: Path) -> "IndexBuilder":
        """
        从 TargetDescriptor YAML 文件创建 IndexBuilder。

        用法:
            builder = IndexBuilder.from_descriptor(Path("src/oracle_ir/targets/px4.yaml"))
        """
        import yaml
        data = yaml.safe_load(descriptor_path.read_text(encoding="utf-8"))
        param_naming = data.get("param_naming", {})

        rules = []
        for tag, patterns in param_naming.items():
            for pat in patterns:
                rules.append((re.compile(pat, re.I), tag))

        # 合并通用规则（目标特定规则优先，通用规则兜底）
        return cls(tag_rules=rules + _GENERIC_TAG_RULES)

    def build(self, blocks: list[SpecBlock]) -> SpecIndex:
        index = SpecIndex()

        for block in blocks:
            bid = block.block_id

            # 实体索引
            index.entity_index.setdefault(block.name, []).append(bid)

            # 类型索引
            index.type_index.setdefault(block.block_type, []).append(bid)

            # 系统索引
            if block.provenance:
                sys_name = block.provenance.source_system
                index.system_index.setdefault(sys_name, []).append(bid)

            # 引用图
            for ref in block.references:
                index.reference_graph.setdefault(ref, []).append(bid)

            # 语义标签
            tags = self._auto_tag(block)
            block.tags = tags
            for tag in tags:
                index.tag_index.setdefault(tag, []).append(bid)

        return index

    def _auto_tag(self, block: SpecBlock) -> list[str]:
        """基于规则自动生成语义标签 — 只检查 name 以避免 NL 中的交叉引用干扰"""
        text = block.name
        tags = set(block.tags or [])
        semantic_tags = block.structured_fields.get("semantic_tags", [])
        if isinstance(semantic_tags, list):
            tags.update(str(tag) for tag in semantic_tags)
        for pattern, tag in self._tag_rules:
            if pattern.search(text):
                tags.add(tag)
        return sorted(tags)
