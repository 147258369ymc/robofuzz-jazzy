"""API 配置管理器 — 从 api_config.yaml 加载 LLM 调用配置"""

from __future__ import annotations
import json
import os
from dataclasses import dataclass
from pathlib import Path
from urllib import error as urllib_error
from urllib import request as urllib_request

import yaml


CONFIG_PATH = Path(__file__).parent / "api_config.yaml"


def _clear_proxy_env():
    """临时清除代理环境变量，避免 httpx 遇到不支持的 socks 代理报错"""
    proxy_keys = ["http_proxy", "https_proxy", "HTTP_PROXY", "HTTPS_PROXY",
                  "all_proxy", "ALL_PROXY"]
    removed = {}
    for key in proxy_keys:
        if key in os.environ:
            removed[key] = os.environ.pop(key)
    return removed


def _restore_proxy_env(removed: dict):
    """恢复代理环境变量"""
    os.environ.update(removed)


@dataclass
class APIConfig:
    provider: str       # "anthropic" | "openai" | "openai_responses"
    base_url: str
    api_key: str
    model: str
    max_tokens: int = 4096
    temperature: float | None = 0.0

    def get_client(self):
        """根据 provider 返回对应的 LLM client"""
        if self.provider == "anthropic":
            return self._anthropic_client()
        elif self.provider == "openai":
            return self._openai_client()
        elif self.provider == "openai_responses":
            raise ValueError("openai_responses uses the raw Responses API, not a client object")
        else:
            raise ValueError(f"未知 provider: {self.provider}")

    def _anthropic_client(self):
        import anthropic
        removed = _clear_proxy_env()
        kwargs = {}
        if self.api_key:
            kwargs["api_key"] = self.api_key
        if self.base_url:
            kwargs["base_url"] = self.base_url
        client = anthropic.Anthropic(**kwargs)
        _restore_proxy_env(removed)
        return client

    def _openai_client(self):
        from openai import OpenAI
        removed = _clear_proxy_env()
        client = OpenAI(
            api_key=self.api_key,
            base_url=self.base_url or None,
        )
        _restore_proxy_env(removed)
        return client

    def call(self, system_prompt: str, user_prompt: str) -> str:
        """统一调用接口，屏蔽 provider 差异"""
        if self.provider == "anthropic":
            client = self._anthropic_client()
            resp = client.messages.create(
                model=self.model,
                max_tokens=self.max_tokens,
                temperature=self.temperature,
                system=system_prompt,
                messages=[{"role": "user", "content": user_prompt}],
            )
            return resp.content[0].text
        elif self.provider == "openai":
            client = self._openai_client()
            resp = client.chat.completions.create(
                model=self.model,
                max_tokens=self.max_tokens,
                temperature=self.temperature,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt},
                ],
            )
            return resp.choices[0].message.content
        elif self.provider == "openai_responses":
            payload = self._responses_payload(
                system_prompt=system_prompt,
                messages=[{"role": "user", "content": user_prompt}],
                tools=[],
            )
            resp = self._openai_responses_create(payload)
            return _openai_responses_text(resp)
        else:
            raise ValueError(f"未知 provider: {self.provider}")

    def call_with_tools(
        self,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
    ) -> dict:
        """
        带 tool_use 的调用接口。

        返回完整的 response 对象（dict 化），包含 content blocks。
        调用方需要自行处理 tool_use / end_turn 的 stop_reason。
        """
        if self.provider == "anthropic":
            client = self._anthropic_client()
            resp = client.messages.create(
                model=self.model,
                max_tokens=self.max_tokens,
                temperature=self.temperature,
                system=system_prompt,
                messages=messages,
                tools=tools,
            )
            # 转为 dict 方便统一处理
            return {
                "stop_reason": resp.stop_reason,
                "content": [_block_to_dict(b) for b in resp.content],
            }
        elif self.provider == "openai":
            # OpenAI function calling 格式转换
            client = self._openai_client()
            oai_tools = [_anthropic_tool_to_openai(t) for t in tools]
            oai_messages = [{"role": "system", "content": system_prompt}]
            oai_messages.extend(_convert_messages_to_openai(messages))
            resp = client.chat.completions.create(
                model=self.model,
                max_tokens=self.max_tokens,
                temperature=self.temperature,
                messages=oai_messages,
                tools=oai_tools if oai_tools else None,
            )
            return _openai_response_to_anthropic(resp)
        elif self.provider == "openai_responses":
            payload = self._responses_payload(
                system_prompt=system_prompt,
                messages=messages,
                tools=tools,
            )
            resp = self._openai_responses_create(payload)
            return _openai_responses_response_to_anthropic(resp)
        else:
            raise ValueError(f"未知 provider: {self.provider}")

    def _responses_payload(
        self,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
    ) -> dict:
        payload: dict = {
            "model": self.model,
            "instructions": system_prompt,
            "input": _convert_messages_to_openai_responses(messages),
            "max_output_tokens": self.max_tokens,
        }
        if self.temperature is not None:
            payload["temperature"] = self.temperature
        if tools:
            payload["tools"] = [_anthropic_tool_to_openai_responses(t) for t in tools]
        return payload

    def _openai_responses_create(self, payload: dict) -> dict:
        """Call an OpenAI Responses-compatible endpoint without requiring openai SDK."""
        if not self.api_key:
            raise ValueError("OPENAI_API_KEY is required for provider=openai_responses")
        url = _join_responses_url(self.base_url)
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        req = urllib_request.Request(
            url,
            data=data,
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with urllib_request.urlopen(req, timeout=180) as resp:
                body = resp.read().decode("utf-8")
        except urllib_error.HTTPError as exc:
            err_body = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"Responses API HTTP {exc.code}: {err_body}") from exc
        except urllib_error.URLError as exc:
            raise RuntimeError(f"Responses API request failed: {exc}") from exc
        return _parse_openai_responses_body(body)


# ============================================================
# 格式转换辅助函数
# ============================================================

def _block_to_dict(block) -> dict:
    """将 Anthropic SDK 的 content block 转为 dict"""
    if block.type == "text":
        return {"type": "text", "text": block.text}
    elif block.type == "tool_use":
        return {
            "type": "tool_use",
            "id": block.id,
            "name": block.name,
            "input": block.input,
        }
    return {"type": block.type}


def _anthropic_tool_to_openai(tool: dict) -> dict:
    """Anthropic tool schema → OpenAI function calling schema"""
    return {
        "type": "function",
        "function": {
            "name": tool["name"],
            "description": tool.get("description", ""),
            "parameters": tool.get("input_schema", {"type": "object", "properties": {}}),
        },
    }


def _anthropic_tool_to_openai_responses(tool: dict) -> dict:
    """Anthropic tool schema → OpenAI Responses function tool schema."""
    return {
        "type": "function",
        "name": tool["name"],
        "description": tool.get("description", ""),
        "parameters": tool.get("input_schema", {"type": "object", "properties": {}}),
    }


def _convert_messages_to_openai(messages: list[dict]) -> list[dict]:
    """Anthropic messages 格式 → OpenAI messages 格式"""
    import json as _json
    result = []
    for msg in messages:
        role = msg["role"]
        content = msg.get("content")
        if isinstance(content, str):
            result.append({"role": role, "content": content})
        elif isinstance(content, list):
            for block in content:
                if block.get("type") == "text":
                    result.append({"role": role, "content": block["text"]})
                elif block.get("type") == "tool_use":
                    result.append({
                        "role": "assistant",
                        "tool_calls": [{
                            "id": block["id"],
                            "type": "function",
                            "function": {
                                "name": block["name"],
                                "arguments": _json.dumps(block["input"]),
                            },
                        }],
                    })
                elif block.get("type") == "tool_result":
                    result.append({
                        "role": "tool",
                        "tool_call_id": block.get("tool_use_id", ""),
                        "content": block.get("content", ""),
                    })
    return result


def _convert_messages_to_openai_responses(messages: list[dict]) -> list[dict]:
    """Anthropic messages 格式 → OpenAI Responses input item 格式."""
    result = []
    for msg in messages:
        role = msg["role"]
        content = msg.get("content")
        if isinstance(content, str):
            result.append({"role": role, "content": content})
            continue
        if not isinstance(content, list):
            continue

        text_parts = []
        for block in content:
            block_type = block.get("type")
            if block_type == "text":
                text_parts.append(block.get("text", ""))
            elif block_type == "tool_use":
                if text_parts:
                    result.append({"role": "assistant", "content": "\n".join(text_parts)})
                    text_parts = []
                result.append({
                    "type": "function_call",
                    "call_id": block["id"],
                    "name": block["name"],
                    "arguments": json.dumps(block.get("input", {}), ensure_ascii=False),
                })
            elif block_type == "tool_result":
                if text_parts:
                    result.append({"role": "user", "content": "\n".join(text_parts)})
                    text_parts = []
                result.append({
                    "type": "function_call_output",
                    "call_id": block.get("tool_use_id", ""),
                    "output": block.get("content", ""),
                })
        if text_parts:
            result.append({"role": role, "content": "\n".join(text_parts)})
    return result


def _openai_response_to_anthropic(resp) -> dict:
    """OpenAI response → Anthropic-style dict"""
    import json as _json
    choice = resp.choices[0]
    msg = choice.message
    content = []
    if msg.content:
        content.append({"type": "text", "text": msg.content})
    if msg.tool_calls:
        for tc in msg.tool_calls:
            content.append({
                "type": "tool_use",
                "id": tc.id,
                "name": tc.function.name,
                "input": _json.loads(tc.function.arguments),
            })
    stop_reason = "tool_use" if msg.tool_calls else "end_turn"
    return {"stop_reason": stop_reason, "content": content}


def _parse_openai_responses_body(body: str) -> dict:
    """Parse either JSON Responses output or RightCode's text/event-stream body."""
    text = body.strip()
    if not text:
        raise RuntimeError("Responses API returned an empty body")
    if text.startswith("{"):
        return json.loads(text)

    output_items: list[dict] = []
    text_done: list[str] = []
    completed_response: dict | None = None

    for raw_event in text.split("\n\n"):
        data_lines = []
        for line in raw_event.splitlines():
            if line.startswith("data:"):
                data_lines.append(line[len("data:"):].strip())
        if not data_lines:
            continue
        data_text = "\n".join(data_lines)
        if data_text == "[DONE]":
            continue
        try:
            event = json.loads(data_text)
        except json.JSONDecodeError:
            continue

        event_type = event.get("type", "")
        if event_type == "response.output_text.done":
            value = event.get("text", "")
            if value:
                text_done.append(value)
        elif event_type == "response.output_item.done":
            item = event.get("item")
            if isinstance(item, dict):
                output_items.append(item)
        elif event_type == "response.completed":
            response = event.get("response")
            if isinstance(response, dict):
                completed_response = response

    if not output_items and completed_response:
        output = completed_response.get("output")
        if isinstance(output, list) and output:
            output_items = output

    if not output_items and text_done:
        output_items = [{
            "type": "message",
            "content": [{"type": "output_text", "text": "".join(text_done)}],
        }]

    output_text = _extract_output_text_from_items(output_items) or "".join(text_done)
    result = dict(completed_response or {})
    result["output"] = output_items
    if output_text:
        result["output_text"] = output_text
    return result


def _openai_responses_response_to_anthropic(resp) -> dict:
    """OpenAI Responses API response → Anthropic-style dict used by the agent loop."""
    content = []
    for item in _as_list(_get_value(resp, "output", [])):
        item_type = _get_value(item, "type")
        if item_type == "message":
            for part in _as_list(_get_value(item, "content", [])):
                part_type = _get_value(part, "type")
                if part_type in {"output_text", "text"}:
                    text = _get_value(part, "text", "")
                    if text:
                        content.append({"type": "text", "text": text})
        elif item_type in {"output_text", "text"}:
            text = _get_value(item, "text", "")
            if text:
                content.append({"type": "text", "text": text})
        elif item_type == "function_call":
            raw_args = _get_value(item, "arguments", "{}") or "{}"
            try:
                args = json.loads(raw_args)
            except json.JSONDecodeError:
                args = {}
            content.append({
                "type": "tool_use",
                "id": _get_value(item, "call_id", _get_value(item, "id", "")),
                "name": _get_value(item, "name", ""),
                "input": args,
            })
    stop_reason = "tool_use" if any(block["type"] == "tool_use" for block in content) else "end_turn"
    return {"stop_reason": stop_reason, "content": content}


def _openai_responses_text(resp) -> str:
    """Extract plain text from a Responses API result."""
    text = _get_value(resp, "output_text", "")
    if text:
        return text
    converted = _openai_responses_response_to_anthropic(resp)
    return "".join(block["text"] for block in converted["content"] if block["type"] == "text")


def _extract_output_text_from_items(items: list[dict]) -> str:
    chunks = []
    for item in items:
        if _get_value(item, "type") != "message":
            continue
        for part in _as_list(_get_value(item, "content", [])):
            if _get_value(part, "type") in {"output_text", "text"}:
                chunks.append(_get_value(part, "text", ""))
    return "".join(chunks)


def _join_responses_url(base_url: str) -> str:
    base = (base_url or "").rstrip("/")
    if not base:
        return "https://api.openai.com/v1/responses"
    if base.endswith("/responses"):
        return base
    return f"{base}/responses"


def _get_value(obj, key: str, default=None):
    if isinstance(obj, dict):
        return obj.get(key, default)
    return getattr(obj, key, default)


def _as_list(value) -> list:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    return list(value)


# ============================================================
# 配置加载
# ============================================================

def load_config(profile: str | None = None) -> APIConfig:
    """
    从 api_config.yaml 加载配置。

    优先级: 函数参数 > 环境变量 ORACLE_API_PROFILE > yaml 中的 active_profile
    """
    if not CONFIG_PATH.exists():
        raise FileNotFoundError(
            f"API 配置文件不存在: {CONFIG_PATH}\n"
            f"请复制 api_config.yaml.example 并填写你的 API 信息"
        )

    data = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    profiles = data.get("profiles", {})

    if profile is None:
        profile = os.environ.get("ORACLE_API_PROFILE", data.get("active_profile"))

    if profile not in profiles:
        available = ", ".join(profiles.keys())
        raise ValueError(f"Profile '{profile}' 不存在。可用: {available}")

    cfg = profiles[profile]

    api_key = cfg.get("api_key", "") or ""
    if not api_key:
        env_key = "ANTHROPIC_API_KEY" if cfg["provider"] == "anthropic" else "OPENAI_API_KEY"
        api_key = os.environ.get(env_key, "")
    if not api_key and cfg["provider"] in {"openai", "openai_responses"}:
        api_key = _load_openai_key_from_codex_auth()

    return APIConfig(
        provider=cfg["provider"],
        base_url=cfg.get("base_url", ""),
        api_key=api_key,
        model=cfg["model"],
        max_tokens=cfg.get("max_tokens", 4096),
        temperature=cfg.get("temperature", 0.0),
    )


def _load_openai_key_from_codex_auth() -> str:
    """Load OPENAI_API_KEY from Codex auth.json when environment variable is absent."""
    auth_path = Path(os.environ.get("ORACLE_CODEX_AUTH_PATH", "~/.codex/auth.json")).expanduser()
    if not auth_path.exists():
        return ""
    try:
        data = json.loads(auth_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return ""
    return str(data.get("OPENAI_API_KEY", "") or "")
