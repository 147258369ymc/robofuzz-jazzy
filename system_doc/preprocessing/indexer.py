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


class IndexBuilder:
    """从 SpecBlock 列表构建 SpecIndex"""

    # 自动语义标签规则（可扩展）
    TAG_RULES: list[tuple[re.Pattern, str]] = [
        (re.compile(r"(MPC_.*VEL|MPC_XY_VEL|FW_AIRSPD_M[AI][XN]|vel_max|vel_min|VEL_MANUAL|XY_VEL_MAX)", re.I), "velocity_constraint"),
        (re.compile(r"(MC_ROLL|MC_PITCH|MC_YAW|MAN_[RPY]_MAX|ATT_.*MAX|TILT_MAX|FW_[RPY]_LIM)", re.I), "attitude_constraint"),
        (re.compile(r"(altitude|height|alt_max|alt_min|climb_rate|sink_rate|MPC_Z_)", re.I), "altitude_constraint"),
        (re.compile(r"(position\s*(max|min|limit|error)|waypoint|pos_max|pos_min|gps\s*(loss|fail))", re.I), "position_constraint"),
        (re.compile(r"(flight.?mode|offboard|manual\s*control|posctl|altctl|stabilized|FLTMODE)", re.I), "flight_mode"),
        (re.compile(r"\b(imu|gyro|accel|baro|mag|lidar|sonar)\b", re.I), "sensor"),
        (re.compile(r"(timeout|timer|delay|interval|deadband)", re.I), "temporal"),
        (re.compile(r"(battery|voltage|current|power|energy)", re.I), "power"),
        (re.compile(r"(geofence|geo_fence|fence_act|GF_)", re.I), "geofence"),
        (re.compile(r"(motor|actuator|servo|pwm|thrust)", re.I), "actuator"),
        (re.compile(r"(failsafe|fail_act|emergency|abort|CBRK)", re.I), "safety"),
    ]

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
        tags = set()
        for pattern, tag in self.TAG_RULES:
            if pattern.search(text):
                tags.add(tag)
        return sorted(tags)[:5]
