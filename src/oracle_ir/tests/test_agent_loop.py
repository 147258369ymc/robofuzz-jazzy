"""Tests for OracleIR tool-use agent loop controls."""

from __future__ import annotations

import os
import unittest
from unittest.mock import patch

from src.oracle_ir.agent import agent_loop


class AgentLoopTests(unittest.TestCase):
    def test_default_max_turns_is_ten(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(10, agent_loop.get_max_turns())

    def test_max_turns_can_be_overridden_by_environment(self):
        with patch.dict(os.environ, {"ORACLE_AGENT_MAX_TURNS": "8"}, clear=True):
            self.assertEqual(8, agent_loop.get_max_turns())

    def test_invalid_max_turns_env_falls_back_to_default(self):
        with patch.dict(os.environ, {"ORACLE_AGENT_MAX_TURNS": "not-a-number"}, clear=True):
            self.assertEqual(10, agent_loop.get_max_turns())

    def test_save_spec_tool_success_completes_task(self):
        class FakeConfig:
            provider = "fake"
            model = "fake-model"

            def call_with_tools(self, system_prompt, messages, tools):
                return {
                    "stop_reason": "tool_use",
                    "content": [{
                        "type": "tool_use",
                        "id": "tool-1",
                        "name": "save_spec",
                        "input": {
                            "filename": "example.yaml",
                            "yaml_content": "id: example",
                        },
                    }],
                }

        class FakeToolExecutor:
            def execute(self, tool_name, tool_input):
                self.tool_name = tool_name
                self.tool_input = tool_input
                return '{"saved": "/tmp/example.yaml", "size": 11}'

        result = agent_loop.run_agent_task(
            "make a spec",
            FakeConfig(),
            FakeToolExecutor(),
            "system prompt",
        )

        self.assertTrue(result["success"])
        self.assertEqual("saved", result["message"])
        self.assertEqual(1, result["turns"])

    def test_transient_api_overload_is_retried(self):
        class FakeConfig:
            provider = "fake"
            model = "fake-model"

            def __init__(self):
                self.calls = 0

            def call_with_tools(self, system_prompt, messages, tools):
                self.calls += 1
                if self.calls == 1:
                    raise RuntimeError("Error code: 503 - system cpu overloaded")
                return {
                    "stop_reason": "tool_use",
                    "content": [{
                        "type": "tool_use",
                        "id": "tool-1",
                        "name": "save_spec",
                        "input": {
                            "filename": "example.yaml",
                            "yaml_content": "id: example",
                        },
                    }],
                }

        class FakeToolExecutor:
            def execute(self, tool_name, tool_input):
                return '{"saved": "/tmp/example.yaml", "size": 11}'

        cfg = FakeConfig()
        with patch.dict(os.environ, {
            "ORACLE_API_MAX_RETRIES": "2",
            "ORACLE_API_RETRY_BASE_SECONDS": "0",
        }, clear=True):
            result = agent_loop.run_agent_task(
                "make a spec",
                cfg,
                FakeToolExecutor(),
                "system prompt",
            )

        self.assertTrue(result["success"])
        self.assertEqual("saved", result["message"])
        self.assertEqual(2, cfg.calls)


if __name__ == "__main__":
    unittest.main()
