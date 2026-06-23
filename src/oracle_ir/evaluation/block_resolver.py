"""Helpers for resolving preprocessed SpecBlocks during OracleIR evaluation."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from src.oracle_ir.schema import OracleIR, Parameter


def load_block_map(blocks_dir: Path) -> dict[str, dict[str, Any]]:
    blocks: dict[str, dict[str, Any]] = {}
    for path in sorted(blocks_dir.glob("*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        blocks[data["block_id"]] = data
    return blocks


def resolve_parameter_block(
    ir: OracleIR,
    param: Parameter,
    blocks_dir: Path,
    blocks: dict[str, dict[str, Any]] | None = None,
) -> dict[str, Any] | None:
    """Resolve a parameter SpecBlock without assuming param.name equals file stem."""
    block_map = blocks if blocks is not None else load_block_map(blocks_dir)

    for provenance in ir.provenance:
        block = block_map.get(provenance.chunk_id)
        if block is not None and _matches_parameter(block, param):
            return block

    exact_matches = [
        block
        for block in block_map.values()
        if block.get("block_type") == "parameter" and _matches_parameter(block, param)
    ]
    if not exact_matches:
        return None

    source_matches = [
        block
        for block in exact_matches
        if (block.get("provenance") or {}).get("source_file") == param.source
    ]
    matches = source_matches or exact_matches
    return sorted(matches, key=lambda block: block["block_id"])[0]


def _matches_parameter(block: dict[str, Any], param: Parameter) -> bool:
    if block.get("block_type") != "parameter":
        return False
    if param.source:
        source_file = (block.get("provenance") or {}).get("source_file")
        if source_file and source_file != param.source:
            return False
    fields = block.get("structured_fields") or {}
    return block.get("name") == param.name or fields.get("path") == param.name
