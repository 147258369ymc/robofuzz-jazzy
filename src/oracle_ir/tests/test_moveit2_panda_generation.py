"""Tests for descriptor-driven MoveIt2 Panda OracleIR generation from Jazzy SpecBlocks."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import yaml

from src.oracle_ir.targets.descriptor import TargetDescriptor
from src.oracle_ir.transform.compiler import compile_oracle_ir
from src.oracle_ir.transform.parser import load_all_specs
from src.oracle_ir.transform.validator import validate_oracle_ir


REPO_ROOT = Path(__file__).resolve().parents[3]


class MoveIt2PandaGenerationTests(unittest.TestCase):
    def test_generator_creates_jazzy_specs_from_preprocessed_blocks(self):
        from src.oracle_ir.agent.generate_from_preprocessed import generate_specs

        with tempfile.TemporaryDirectory() as td:
            out_dir = Path(td)
            generated = generate_specs(target="moveit2_panda", repo_root=REPO_ROOT, output_dir=out_dir)

            self.assertEqual(27, len(generated))
            velocity_path = out_dir / "moveit2_panda_range_panda_joint1_max_velocity.yaml"
            acceleration_path = out_dir / "moveit2_panda_range_panda_joint1_max_acceleration.yaml"
            self.assertTrue(velocity_path.exists())
            self.assertTrue(acceleration_path.exists())

            velocity = yaml.safe_load(velocity_path.read_text(encoding="utf-8"))
            self.assertEqual("moveit2_panda.range.panda_joint1_max_velocity", velocity["id"])
            self.assertEqual("range_bound", velocity["type"])
            self.assertEqual("moveit2-2.12.4_panda-3.1.0_jazzy", velocity["version"])
            self.assertEqual("/joint_states", velocity["observations"][0]["topic"])
            self.assertEqual("velocity", velocity["observations"][0]["field"])
            self.assertEqual(0, velocity["observations"][0]["index"])
            self.assertEqual(2.175, velocity["parameters"][0]["default"])
            self.assertEqual(
                "system_doc/moveit2_panda/config/joint_limits.yaml",
                velocity["provenance"][0]["source_file"],
            )
            self.assertNotIn("/panda_arm_controller/state", velocity_path.read_text(encoding="utf-8"))

            acceleration = yaml.safe_load(acceleration_path.read_text(encoding="utf-8"))
            self.assertEqual("sequential_pairs", acceleration["window"]["type"])
            self.assertEqual(3.75, acceleration["parameters"][0]["default"])
            self.assertEqual(
                "system_doc/moveit2_panda/config/joint_limits.yaml",
                acceleration["provenance"][0]["source_file"],
            )

    def test_generation_rules_are_descriptor_driven(self):
        descriptor_path = REPO_ROOT / "src/oracle_ir/targets/moveit2_panda.yaml"
        descriptor = yaml.safe_load(descriptor_path.read_text(encoding="utf-8"))
        rules = descriptor.get("oracle_generation", {})
        profiles = rules.get("profiles", [])

        self.assertTrue(profiles)
        self.assertEqual("generic_joint_limits_v1", profiles[0]["output_profile"])
        self.assertEqual("/joint_states", profiles[0]["state_topic"])
        self.assertEqual("panda_joint1", profiles[0]["joints"][0]["name"])

        generator_source = (REPO_ROOT / "src/oracle_ir/agent/generate_from_preprocessed.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("panda_joint", generator_source)
        self.assertNotIn("moveit2_panda", generator_source)
        self.assertNotIn("joint_limits.yaml", generator_source)

    def test_generated_specs_validate_and_compile(self):
        from src.oracle_ir.agent.generate_from_preprocessed import generate_specs

        with tempfile.TemporaryDirectory() as td:
            out_dir = Path(td)
            generate_specs(target="moveit2_panda", repo_root=REPO_ROOT, output_dir=out_dir)
            specs = load_all_specs(out_dir)

            self.assertEqual(27, len(specs))
            for spec in specs:
                result = validate_oracle_ir(spec)
                self.assertTrue(result.valid, f"{spec.id}: {result.errors}")
                compile_oracle_ir(spec)

    def test_generated_specs_pass_existing_evaluation_gates(self):
        from src.oracle_ir.agent.generate_from_preprocessed import generate_specs
        from src.oracle_ir.evaluation.run_eval import run_evaluation

        with tempfile.TemporaryDirectory() as td:
            out_dir = Path(td)
            generate_specs(target="moveit2_panda", repo_root=REPO_ROOT, output_dir=out_dir)
            report = run_evaluation(spec_dir=out_dir, target="moveit2_panda")
        by_name = {dimension.name: dimension for dimension in report.dimensions}

        provenance = by_name["D1: Provenance Accuracy"]
        semantics = by_name["D2: Semantic Accuracy"]

        self.assertEqual(provenance.total, provenance.passed, provenance.failures[:5])
        self.assertEqual(semantics.total, semantics.passed, semantics.failures[:5])

    def test_moveit_descriptor_uses_jazzy_runtime_topics(self):
        descriptor = TargetDescriptor.load(REPO_ROOT / "src/oracle_ir/targets/moveit2_panda.yaml")
        topics = {topic.name for topic in descriptor.topics}

        self.assertEqual("moveit2-2.12.4_panda-3.1.0_jazzy", descriptor.version)
        self.assertIn("/panda_arm_controller/controller_state", topics)
        self.assertNotIn("/panda_arm_controller/state", topics)


if __name__ == "__main__":
    unittest.main()
