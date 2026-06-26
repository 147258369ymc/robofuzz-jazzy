"""Regression tests for portable OracleIR generation rules."""

from __future__ import annotations

import unittest
from pathlib import Path

import yaml

from src_jazzy.oracle_ir.targets.descriptor import TargetDescriptor
from src_jazzy.oracle_ir.transform.parser import _dict_to_oracle_ir
from src_jazzy.oracle_ir.transform.validator import validate_oracle_ir


REPO_ROOT = Path(__file__).resolve().parents[3]


class OracleIRGenerationRuleTests(unittest.TestCase):
    def test_parser_preserves_generic_operating_modes_scope(self):
        ir = _dict_to_oracle_ir({
            "id": "turtlebot4_jazzy.range.cmd_linear_x",
            "type": "range_bound",
            "system": "turtlebot4_jazzy",
            "scope": {"operating_modes": ["safety_enabled_motion"]},
            "observations": [
                {
                    "name": "cmd_linear_x",
                    "topic": "/cmd_vel",
                    "field": "twist.linear.x",
                    "unit": "m/s",
                }
            ],
            "assertions": [
                {"expr": "abs(cmd_linear_x) <= 0.46 + tolerance"}
            ],
        })

        self.assertEqual(["safety_enabled_motion"], ir.scope.operating_modes)

    def test_validator_warns_when_remaining_margin_is_maximized(self):
        ir = _dict_to_oracle_ir({
            "id": "turtlebot4_jazzy.range.cmd_linear_x",
            "type": "range_bound",
            "system": "turtlebot4_jazzy",
            "observations": [
                {
                    "name": "cmd_linear_x",
                    "topic": "/cmd_vel",
                    "field": "twist.linear.x",
                    "unit": "m/s",
                }
            ],
            "parameters": [
                {
                    "name": "linear.x.max_velocity",
                    "source": "unit-test",
                    "unit": "m/s",
                    "default": 0.46,
                }
            ],
            "assertions": [
                {
                    "expr": "abs(cmd_linear_x) <= param('linear.x.max_velocity') + tolerance"
                }
            ],
            "feedback": [
                {
                    "name": "cmd_linear_x_margin",
                    "metric": "param('linear.x.max_velocity') - abs(cmd_linear_x)",
                    "direction": "maximize",
                }
            ],
        })

        result = validate_oracle_ir(ir)

        self.assertTrue(result.valid)
        self.assertTrue(
            any("remaining margin" in warning for warning in result.warnings),
            result.warnings,
        )

    def test_validator_accepts_minimized_remaining_margin_feedback(self):
        ir = _dict_to_oracle_ir({
            "id": "turtlebot4_jazzy.range.cmd_linear_x",
            "type": "range_bound",
            "system": "turtlebot4_jazzy",
            "observations": [
                {
                    "name": "cmd_linear_x",
                    "topic": "/cmd_vel",
                    "field": "twist.linear.x",
                    "unit": "m/s",
                }
            ],
            "parameters": [
                {
                    "name": "linear.x.max_velocity",
                    "source": "unit-test",
                    "unit": "m/s",
                    "default": 0.46,
                }
            ],
            "assertions": [
                {
                    "expr": "abs(cmd_linear_x) <= param('linear.x.max_velocity') + tolerance"
                }
            ],
            "feedback": [
                {
                    "name": "cmd_linear_x_margin",
                    "metric": "param('linear.x.max_velocity') - abs(cmd_linear_x)",
                    "direction": "minimize",
                }
            ],
        })

        result = validate_oracle_ir(ir)

        self.assertTrue(result.valid)
        self.assertFalse(
            any("remaining margin" in warning for warning in result.warnings),
            result.warnings,
        )

    def test_turtlebot4_descriptor_exports_generation_rules(self):
        descriptor = TargetDescriptor.load(
            REPO_ROOT / "src_jazzy/oracle_ir/targets/turtlebot4_jazzy.yaml"
        )

        context = descriptor.to_agent_context()

        self.assertIn("## OracleIR 生成规则", context)
        self.assertIn("加速度", context)
        self.assertIn("相邻样本", context)
        self.assertIn("/cmd_vel", context)

    def test_turtlebot4_specs_validate_against_profile_watchlist(self):
        spec_dir = REPO_ROOT / "src_jazzy/oracle_ir/specs/turtlebot4_jazzy"
        profile = yaml.safe_load(
            (REPO_ROOT / "target_profiles/turtlebot4_jazzy.yaml").read_text(
                encoding="utf-8"
            )
        )
        watchlist = dict(profile["watch"]["required"])
        watchlist.update(profile["watch"].get("optional", {}))

        invalid = {}
        for path in sorted(spec_dir.glob("*.yaml")):
            ir = _dict_to_oracle_ir(yaml.safe_load(path.read_text(encoding="utf-8")))
            result = validate_oracle_ir(ir, watchlist=watchlist)
            if not result.valid:
                invalid[path.name] = result.errors

        self.assertEqual({}, invalid)


if __name__ == "__main__":
    unittest.main()
