"""API 配置管理器 — 从 api_config.yaml 加载 LLM 调用配置"""

from __future__ import annotations
import os
from dataclasses import dataclass
from pathlib import Path

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
    provider: str       # "anthropic" | "openai"
    base_url: str
    api_key: str
    model: str
    max_tokens: int = 4096
    temperature: float = 0.0

    def get_client(self):
        """根据 provider 返回对应的 LLM client"""
        if self.provider == "anthropic":
            return self._anthropic_client()
        elif self.provider == "openai":
            return self._openai_client()
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
        else:
            raise ValueError(f"未知 provider: {self.provider}")


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

    return APIConfig(
        provider=cfg["provider"],
        base_url=cfg.get("base_url", ""),
        api_key=api_key,
        model=cfg["model"],
        max_tokens=cfg.get("max_tokens", 4096),
        temperature=cfg.get("temperature", 0.0),
    )
