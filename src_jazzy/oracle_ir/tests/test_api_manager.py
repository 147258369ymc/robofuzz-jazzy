"""Tests for Jazzy OracleIR agent API provider configuration and adapters."""

from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from src_jazzy.oracle_ir.agent.api_manager import (
    APIConfig,
    _openai_responses_response_to_anthropic,
    _parse_openai_responses_body,
    load_config,
)


class JazzyAPIManagerTests(unittest.TestCase):
    def test_default_profile_uses_rightcode_gpt_responses_api(self):
        with tempfile.TemporaryDirectory() as td:
            auth_path = Path(td) / "auth.json"
            auth_path.write_text(
                json.dumps({"OPENAI_API_KEY": "test-rightcode-key"}),
                encoding="utf-8",
            )

            with patch.dict(
                os.environ,
                {"ORACLE_CODEX_AUTH_PATH": str(auth_path)},
                clear=True,
            ):
                cfg = load_config()

        self.assertEqual("openai_responses", cfg.provider)
        self.assertEqual("https://right.codes/codex/v1", cfg.base_url)
        self.assertEqual("gpt-5.5", cfg.model)
        self.assertEqual("test-rightcode-key", cfg.api_key)

    def test_openai_responses_tool_call_converts_to_agent_tool_use(self):
        response = {
            "output": [
                {
                    "type": "message",
                    "content": [
                        {"type": "output_text", "text": "need to inspect a block"}
                    ],
                },
                {
                    "type": "function_call",
                    "call_id": "call_123",
                    "name": "read_block",
                    "arguments": json.dumps({"block_id": "tb4.block"}),
                },
            ],
        }

        converted = _openai_responses_response_to_anthropic(response)

        self.assertEqual("tool_use", converted["stop_reason"])
        self.assertEqual(
            {"type": "text", "text": "need to inspect a block"},
            converted["content"][0],
        )
        self.assertEqual(
            {
                "type": "tool_use",
                "id": "call_123",
                "name": "read_block",
                "input": {"block_id": "tb4.block"},
            },
            converted["content"][1],
        )

    def test_openai_responses_sse_body_parses_output_text(self):
        body = (
            'event: response.output_text.done\n'
            'data: {"type":"response.output_text.done","text":"pong"}\n\n'
            'event: response.completed\n'
            'data: {"type":"response.completed","response":{"output":[]}}\n\n'
        )

        parsed = _parse_openai_responses_body(body)

        self.assertEqual("pong", parsed["output_text"])
        self.assertEqual(
            [
                {
                    "type": "message",
                    "content": [{"type": "output_text", "text": "pong"}],
                }
            ],
            parsed["output"],
        )

    def test_openai_responses_sse_body_parses_function_call(self):
        body = (
            'event: response.output_item.done\n'
            'data: {"type":"response.output_item.done","item":{"type":"function_call",'
            '"call_id":"call_789","name":"read_block",'
            '"arguments":"{\\\"block_id\\\": \\\"tb4.block\\\"}"}}\n\n'
        )

        parsed = _parse_openai_responses_body(body)
        converted = _openai_responses_response_to_anthropic(parsed)

        self.assertEqual(
            {
                "type": "tool_use",
                "id": "call_789",
                "name": "read_block",
                "input": {"block_id": "tb4.block"},
            },
            converted["content"][0],
        )

    def test_openai_responses_call_with_tools_uses_responses_payload(self):
        cfg = APIConfig(
            provider="openai_responses",
            base_url="https://right.codes/codex/v1",
            api_key="test-key",
            model="gpt-5.5",
            temperature=None,
        )
        api_response = {
            "output": [
                {
                    "type": "function_call",
                    "call_id": "call_456",
                    "name": "list_specs",
                    "arguments": "{}",
                }
            ]
        }

        with patch.object(cfg, "_openai_responses_create", return_value=api_response) as create:
            converted = cfg.call_with_tools(
                system_prompt="system",
                messages=[{"role": "user", "content": "make oracle"}],
                tools=[
                    {
                        "name": "list_specs",
                        "description": "List specs",
                        "input_schema": {"type": "object", "properties": {}},
                    }
                ],
            )

        payload = create.call_args.args[0]
        self.assertEqual("gpt-5.5", payload["model"])
        self.assertEqual("system", payload["instructions"])
        self.assertEqual([{"role": "user", "content": "make oracle"}], payload["input"])
        self.assertEqual(
            [
                {
                    "type": "function",
                    "name": "list_specs",
                    "description": "List specs",
                    "parameters": {"type": "object", "properties": {}},
                }
            ],
            payload["tools"],
        )
        self.assertNotIn("temperature", payload)
        self.assertEqual("tool_use", converted["stop_reason"])


if __name__ == "__main__":
    unittest.main()
