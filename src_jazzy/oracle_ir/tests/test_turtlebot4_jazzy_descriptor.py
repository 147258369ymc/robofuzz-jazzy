"""Tests for the Jazzy TurtleBot4 target descriptor used by OracleIR agent generation."""

from __future__ import annotations

import unittest
from pathlib import Path

from src_jazzy.oracle_ir.targets.descriptor import TargetDescriptor


REPO_ROOT = Path(__file__).resolve().parents[3]


class TurtleBot4JazzyDescriptorTests(unittest.TestCase):
    def test_descriptor_exposes_runtime_topics_and_semantic_tags(self):
        descriptor = TargetDescriptor.load(
            REPO_ROOT / "src_jazzy/oracle_ir/targets/turtlebot4_jazzy.yaml"
        )
        topics = {topic.name: topic for topic in descriptor.topics}
        tag_rules = dict(descriptor.get_tag_rules())

        self.assertEqual("turtlebot4_jazzy", descriptor.name)
        self.assertEqual("TurtleBot4 Jazzy Standard + Create3", descriptor.display_name)
        self.assertIn("/cmd_vel", topics)
        self.assertEqual("geometry_msgs/msg/TwistStamped", topics["/cmd_vel"].msg_type)
        self.assertIn("/odom", topics)
        self.assertIn("/scan", topics)
        self.assertIn("/hazard_detection", topics)
        self.assertIn("/wheel_vels", topics)

        self.assertIn("cmd_odom_consistency", tag_rules.values())
        self.assertIn("scan_range_constraint", tag_rules.values())
        self.assertIn("hazard_reflex_constraint", tag_rules.values())
        self.assertIn("wheel_velocity_constraint", tag_rules.values())

    def test_descriptor_agent_context_mentions_create3_safety_scope(self):
        descriptor = TargetDescriptor.load(
            REPO_ROOT / "src_jazzy/oracle_ir/targets/turtlebot4_jazzy.yaml"
        )
        context = descriptor.to_agent_context()

        self.assertIn("safety_enabled_motion", context)
        self.assertIn("/hazard_detection", context)
        self.assertIn("/cmd_vel", context)


if __name__ == "__main__":
    unittest.main()
