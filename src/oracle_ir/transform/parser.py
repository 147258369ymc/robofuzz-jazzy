"""OracleIR parser — YAML 文件加载与反序列化"""

from __future__ import annotations
from pathlib import Path
from typing import Any

import yaml

from ..schema import (
    OracleIR, Observation, Parameter, Constant, DerivedVar,
    Assertion, Scope, Window, FeedbackSpec, ProvenanceRef,
)


def _parse_observations(data: list[dict]) -> list[Observation]:
    return [Observation(
        name=d["name"], topic=d["topic"], field=d["field"],
        unit=d.get("unit", ""), index=d.get("index"),
    ) for d in (data or [])]


def _parse_parameters(data: list[dict]) -> list[Parameter]:
    return [Parameter(
        name=d["name"], source=d.get("source", ""),
        unit=d.get("unit", ""), default=d.get("default"),
    ) for d in (data or [])]


def _parse_constants(data: list[dict]) -> list[Constant]:
    return [Constant(
        name=d["name"], value=d["value"], unit=d.get("unit", ""),
    ) for d in (data or [])]


def _parse_derived(data: list[dict]) -> list[DerivedVar]:
    return [DerivedVar(
        name=d["name"], expr=d["expr"], unit=d.get("unit", ""),
    ) for d in (data or [])]


def _parse_assertions(data) -> list[Assertion]:
    if not data:
        return []
    if isinstance(data, dict):
        data = [data]
    return [Assertion(
        expr=d["expr"], tolerance=d.get("tolerance", 0.0),
        severity=d.get("severity", "error"), message=d.get("message", ""),
    ) for d in data]


def _parse_scope(data: dict | None) -> Scope:
    if not data:
        return Scope()

    filter_expr = data.get("filter_expr", "")
    filter_obs_raw = data.get("filter_observations", [])
    filter_observations = _parse_observations(filter_obs_raw)
    require_airborne = data.get("require_airborne", False)

    # 向后兼容：require_airborne → 通用 filter_expr
    if require_airborne and not filter_expr:
        filter_expr = "dist_bottom >= 0.15"
        # 自动注入 filter observation（如果用户未显式声明）
        if not filter_observations:
            filter_observations = [Observation(
                name="dist_bottom",
                topic="/VehicleLocalPosition_PubSubTopic",
                field="dist_bottom",
                unit="m",
            )]

    return Scope(
        flight_modes=data.get("flight_modes", []),
        vehicle_type=data.get("vehicle_type", ""),
        require_airborne=require_airborne,
        preconditions=data.get("preconditions", []),
        filter_expr=filter_expr,
        filter_observations=filter_observations,
    )


def _parse_window(data: dict | None) -> Window:
    if not data:
        return Window()
    return Window(
        type=data.get("type", "every_sample"),
        size=data.get("size", 0.0),
        filter=data.get("filter", ""),
    )


def _parse_feedback(data) -> list[FeedbackSpec]:
    if not data:
        return []
    if isinstance(data, dict):
        data = [data]
    return [FeedbackSpec(
        name=d["name"], metric=d["metric"],
        direction=d.get("direction", "maximize"),
        min_threshold=d.get("min_threshold"),
        target_value=d.get("target_value"),
    ) for d in data]


def _parse_provenance(data: list[dict] | None) -> list[ProvenanceRef]:
    return [ProvenanceRef(
        chunk_id=d.get("chunk_id", ""),
        source_file=d.get("source_file", ""),
        evidence=d.get("evidence", ""),
    ) for d in (data or [])]


def load_oracle_ir(path: str | Path) -> OracleIR:
    """从 YAML 文件加载单个 OracleIR 实例"""
    path = Path(path)
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return _dict_to_oracle_ir(data)


def load_all_specs(spec_dir: str | Path) -> list[OracleIR]:
    """加载目录下所有 YAML spec 文件"""
    spec_dir = Path(spec_dir)
    specs = []
    for yaml_file in sorted(spec_dir.glob("**/*.yaml")):
        specs.append(load_oracle_ir(yaml_file))
    return specs


def _dict_to_oracle_ir(data: dict) -> OracleIR:
    """将字典转换为 OracleIR 对象"""
    return OracleIR(
        id=data["id"],
        type=data["type"],
        system=data["system"],
        version=data.get("version", ""),
        scope=_parse_scope(data.get("scope")),
        observations=_parse_observations(data.get("observations")),
        parameters=_parse_parameters(data.get("parameters")),
        constants=_parse_constants(data.get("constants")),
        derived=_parse_derived(data.get("derived")),
        assertions=_parse_assertions(data.get("assertions", data.get("assertion"))),
        window=_parse_window(data.get("window")),
        feedback=_parse_feedback(data.get("feedback")),
        provenance=_parse_provenance(data.get("provenance")),
    )
