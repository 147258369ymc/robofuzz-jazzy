#!/usr/bin/env python3
"""Descriptor-driven OracleIR generation from preprocessed SpecBlocks.

This module keeps target facts in target descriptor YAML files.  The generator
only implements reusable generation profiles and reads all topics, joints,
indices, source priorities, and units from the descriptor.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import yaml

from src.oracle_ir.agent.generate import load_blocks
from src.oracle_ir.targets.descriptor import TargetDescriptor


PROJECT_ROOT = Path(__file__).resolve().parents[3]


def _format_template(template: str | None, **values: Any) -> str | None:
    if template is None:
        return None
    return template.format(**values)


def _block_matches(
    block: dict[str, Any],
    *,
    block_type: str | None = None,
    name: str | None = None,
    path: str | None = None,
    source_file: str | None = None,
) -> bool:
    if block_type is not None and block.get("block_type") != block_type:
        return False
    if name is not None and block.get("name") != name:
        return False
    fields = block.get("structured_fields") or {}
    if path is not None and fields.get("path") != path:
        return False
    provenance = block.get("provenance") or {}
    if source_file is not None and provenance.get("source_file") != source_file:
        return False
    return True


def _find_block(
    blocks: dict[str, dict[str, Any]],
    source: dict[str, Any],
    *,
    joint: str,
    field: str | None = None,
) -> dict[str, Any]:
    name = _format_template(source.get("name_template"), joint=joint, field=field or "")
    path = _format_template(source.get("path_template"), joint=joint, field=field or "")
    matches = [
        block
        for block in blocks.values()
        if _block_matches(
            block,
            block_type=source.get("block_type"),
            name=name,
            path=path,
            source_file=source.get("source_file"),
        )
    ]
    if not matches:
        raise KeyError(
            "No preprocessed block matched "
            f"source={source!r}, joint={joint!r}, field={field!r}"
        )
    return sorted(matches, key=lambda block: block["block_id"])[0]


def _provenance(block: dict[str, Any], evidence: str) -> dict[str, str]:
    provenance = block.get("provenance") or {}
    return {
        "chunk_id": block.get("block_id", ""),
        "source_file": provenance.get("source_file", ""),
        "evidence": evidence,
    }


def _filename_for(spec: dict[str, Any]) -> str:
    return spec["id"].replace(".", "_").replace("/", "_") + ".yaml"


def _base_spec(descriptor: TargetDescriptor, spec_id: str, scope: dict[str, Any]) -> dict[str, Any]:
    return {
        "id": spec_id,
        "type": "range_bound",
        "system": descriptor.name,
        "version": descriptor.version,
        "scope": scope,
    }


def _observation(
    profile: dict[str, Any],
    joint: dict[str, Any],
    *,
    name: str,
    state_field: str,
    unit: str,
) -> dict[str, Any]:
    return {
        "name": name,
        "topic": profile["state_topic"],
        "field": profile["state_fields"][state_field],
        "index": int(joint["index"]),
        "unit": unit,
    }


def _numeric_field(block: dict[str, Any], field_name: str) -> float:
    fields = block.get("structured_fields") or {}
    value = fields[field_name]
    return float(value)


def _parameter_default(block: dict[str, Any]) -> float:
    fields = block.get("structured_fields") or {}
    if "default" in fields:
        return float(fields["default"])
    return float(fields["value"])


def _parameter_name(block: dict[str, Any]) -> str:
    fields = block.get("structured_fields") or {}
    return str(fields.get("path") or block["name"])


def _build_position_spec(
    descriptor: TargetDescriptor,
    profile: dict[str, Any],
    blocks: dict[str, dict[str, Any]],
    joint: dict[str, Any],
    check: dict[str, Any],
) -> dict[str, Any]:
    source = profile["sources"][check["source"]]
    block = _find_block(blocks, source, joint=joint["name"])
    lower = _numeric_field(block, check["lower_field"])
    upper = _numeric_field(block, check["upper_field"])
    alias = joint["alias"]
    obs_name = f"{alias}_position"
    lower_name = f"{alias}_lower"
    upper_name = f"{alias}_upper"
    unit = joint["position_unit"]

    spec = _base_spec(
        descriptor,
        f"{descriptor.name}.range.{joint['name']}_position_limits",
        profile.get("scope", {}),
    )
    spec.update({
        "observations": [_observation(profile, joint, name=obs_name, state_field="position", unit=unit)],
        "constants": [
            {"name": lower_name, "value": lower, "unit": unit},
            {"name": upper_name, "value": upper, "unit": unit},
        ],
        "assertions": [{
            "expr": f"{obs_name} >= {lower_name} - tolerance and {obs_name} <= {upper_name} + tolerance",
            "tolerance": float(check.get("tolerance", 0.0)),
            "severity": "error",
            "message": f"{joint['name']} position out of configured hard limits",
        }],
        "window": {"type": "every_sample"},
        "feedback": [{
            "name": f"{alias}_position_limit_margin",
            "metric": f"min({obs_name} - {lower_name}, {upper_name} - {obs_name})",
            "direction": "maximize",
        }],
        "provenance": [_provenance(block, f"hard position limits: [{lower}, {upper}] {unit}")],
    })
    return spec


def _build_velocity_spec(
    descriptor: TargetDescriptor,
    profile: dict[str, Any],
    blocks: dict[str, dict[str, Any]],
    joint: dict[str, Any],
    check: dict[str, Any],
) -> dict[str, Any]:
    field = check["source_field"]
    source = profile["sources"][check["source"]]
    block = _find_block(blocks, source, joint=joint["name"], field=field)
    default = _parameter_default(block)
    param_name = _parameter_name(block)
    alias = joint["alias"]
    obs_name = f"{alias}_velocity"
    unit = joint["velocity_unit"]

    spec = _base_spec(
        descriptor,
        f"{descriptor.name}.range.{joint['name']}_max_velocity",
        profile.get("scope", {}),
    )
    spec.update({
        "observations": [_observation(profile, joint, name=obs_name, state_field="velocity", unit=unit)],
        "parameters": [{
            "name": param_name,
            "source": source.get("source_file", ""),
            "unit": unit,
            "default": default,
        }],
        "assertions": [{
            "expr": f"abs({obs_name}) <= param('{param_name}') + tolerance",
            "tolerance": float(check.get("tolerance", 0.0)),
            "severity": "error",
            "message": f"{joint['name']} velocity exceeds configured maximum",
        }],
        "window": {"type": "every_sample"},
        "feedback": [{
            "name": f"{alias}_velocity_margin",
            "metric": f"param('{param_name}') - abs({obs_name})",
            "direction": "maximize",
        }],
        "provenance": [_provenance(block, f"max velocity: {default} {unit}")],
    })
    return spec


def _build_acceleration_spec(
    descriptor: TargetDescriptor,
    profile: dict[str, Any],
    blocks: dict[str, dict[str, Any]],
    joint: dict[str, Any],
    check: dict[str, Any],
) -> dict[str, Any]:
    field = check["source_field"]
    source = profile["sources"][check["source"]]
    block = _find_block(blocks, source, joint=joint["name"], field=field)
    default = _parameter_default(block)
    param_name = _parameter_name(block)
    alias = joint["alias"]
    velocity_name = f"{alias}_velocity"
    acceleration_name = f"{alias}_acceleration"

    spec = _base_spec(
        descriptor,
        f"{descriptor.name}.range.{joint['name']}_max_acceleration",
        profile.get("scope", {}),
    )
    spec.update({
        "observations": [
            _observation(
                profile,
                joint,
                name=velocity_name,
                state_field="velocity",
                unit=joint["velocity_unit"],
            )
        ],
        "parameters": [{
            "name": param_name,
            "source": source.get("source_file", ""),
            "unit": joint["acceleration_unit"],
            "default": default,
        }],
        "derived": [{
            "name": acceleration_name,
            "expr": f"({velocity_name} - prev_{velocity_name}) / dt",
            "unit": joint["acceleration_unit"],
        }],
        "assertions": [{
            "expr": f"abs({acceleration_name}) <= param('{param_name}') + tolerance",
            "tolerance": float(check.get("tolerance", 0.0)),
            "severity": "error",
            "message": f"{joint['name']} acceleration exceeds configured maximum",
        }],
        "window": {"type": "sequential_pairs"},
        "feedback": [{
            "name": f"{alias}_acceleration_abs",
            "metric": f"abs({acceleration_name})",
            "direction": "maximize",
        }],
        "provenance": [_provenance(block, f"max acceleration: {default} {joint['acceleration_unit']}")],
    })
    return spec


def build_specs(
    descriptor: TargetDescriptor,
    blocks: dict[str, dict[str, Any]],
    *,
    output_profile: str | None = None,
) -> list[dict[str, Any]]:
    profiles = descriptor.oracle_generation.get("profiles", [])
    if output_profile is None:
        if not profiles:
            raise ValueError(f"Descriptor {descriptor.name!r} has no oracle_generation profiles")
        profile = profiles[0]
    else:
        matches = [item for item in profiles if item.get("output_profile") == output_profile]
        if not matches:
            raise ValueError(f"Descriptor {descriptor.name!r} has no generation profile {output_profile!r}")
        profile = matches[0]

    if profile.get("output_profile") != "generic_joint_limits_v1":
        raise ValueError(f"Unsupported generation profile: {profile.get('output_profile')!r}")

    builders = {
        "joint_position_hard_bounds": _build_position_spec,
        "joint_velocity_max": _build_velocity_spec,
        "joint_acceleration_max": _build_acceleration_spec,
    }

    specs: list[dict[str, Any]] = []
    for joint in profile.get("joints", []):
        for check in profile.get("checks", []):
            builder = builders.get(check.get("kind"))
            if builder is None:
                raise ValueError(f"Unsupported generated check kind: {check.get('kind')!r}")
            specs.append(builder(descriptor, profile, blocks, joint, check))
    return specs


def generate_specs(
    *,
    target: str,
    repo_root: Path | None = None,
    output_dir: Path | None = None,
    preprocessed_dir: Path | None = None,
    output_profile: str | None = None,
) -> list[Path]:
    root = repo_root or PROJECT_ROOT
    descriptor = TargetDescriptor.load(root / "src" / "oracle_ir" / "targets" / f"{target}.yaml")
    pre_dir = preprocessed_dir or (root / "system_doc" / "preprocessed" / target)
    out_dir = output_dir or (root / "src" / "oracle_ir" / "specs" / target)

    blocks = load_blocks(pre_dir / "blocks")
    specs = build_specs(descriptor, blocks, output_profile=output_profile)

    out_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    for spec in specs:
        path = out_dir / _filename_for(spec)
        path.write_text(yaml.safe_dump(spec, sort_keys=False, allow_unicode=True), encoding="utf-8")
        written.append(path)
    return written


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate OracleIR specs from preprocessed blocks")
    parser.add_argument("--target", required=True, help="Target descriptor name")
    parser.add_argument("--output", type=Path, default=None, help="Output directory")
    parser.add_argument("--preprocessed", type=Path, default=None, help="Preprocessed target directory")
    parser.add_argument("--profile", default=None, help="oracle_generation output_profile")
    args = parser.parse_args()

    paths = generate_specs(
        target=args.target,
        output_dir=args.output,
        preprocessed_dir=args.preprocessed,
        output_profile=args.profile,
    )
    print(f"Generated {len(paths)} OracleIR specs for {args.target}")
    for path in paths:
        print(path)


if __name__ == "__main__":
    main()
