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

    # 确定使用哪个 profile
    if profile is None:
        profile = os.environ.get("ORACLE_API_PROFILE", data.get("active_profile"))

    if profile not in profiles:
        available = ", ".join(profiles.keys())
        raise ValueError(f"Profile '{profile}' 不存在。可用: {available}")

    cfg = profiles[profile]

    # 支持环境变量覆盖 api_key（安全考虑）
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
