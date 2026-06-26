"""Agent Loop — 基于 tool_use 的自主 OracleIR 生成循环

与 simple 模式的区别：
  - Agent 自己决定查询哪些 block、读取哪些参考
  - Agent 自己调用 validator 检查输出
  - Agent 根据错误自主修复，无需硬编码重试逻辑
  - 支持批量生成时的上下文积累（已生成的 spec 可被后续任务参考）
"""

from __future__ import annotations
import json
import logging
import os
import time
from pathlib import Path
from typing import Any

from .api_manager import load_config, APIConfig
from .tools import TOOL_DEFINITIONS, ToolExecutor

logger = logging.getLogger(__name__)

DEFAULT_MAX_TURNS = 10  # 单个任务默认最大对话轮次，防止坏 block 拖住整批
MAX_TURNS_ENV = "ORACLE_AGENT_MAX_TURNS"
DEFAULT_API_MAX_RETRIES = 5
DEFAULT_API_RETRY_BASE_SECONDS = 20.0
API_MAX_RETRIES_ENV = "ORACLE_API_MAX_RETRIES"
API_RETRY_BASE_SECONDS_ENV = "ORACLE_API_RETRY_BASE_SECONDS"


def get_max_turns() -> int:
    """Return per-block agent turn limit, optionally overridden for experiments."""
    raw = os.environ.get(MAX_TURNS_ENV)
    if not raw:
        return DEFAULT_MAX_TURNS
    try:
        value = int(raw)
    except ValueError:
        logger.warning("%s=%r is invalid; using %s", MAX_TURNS_ENV, raw, DEFAULT_MAX_TURNS)
        return DEFAULT_MAX_TURNS
    if value <= 0:
        logger.warning("%s=%r must be positive; using %s", MAX_TURNS_ENV, raw, DEFAULT_MAX_TURNS)
        return DEFAULT_MAX_TURNS
    return value


def get_api_max_retries() -> int:
    """Return slow retry count for transient overload/rate errors."""
    raw = os.environ.get(API_MAX_RETRIES_ENV)
    if raw is None:
        return DEFAULT_API_MAX_RETRIES
    try:
        value = int(raw)
    except ValueError:
        logger.warning("%s=%r is invalid; using %s", API_MAX_RETRIES_ENV, raw, DEFAULT_API_MAX_RETRIES)
        return DEFAULT_API_MAX_RETRIES
    if value < 0:
        logger.warning("%s=%r must be non-negative; using %s", API_MAX_RETRIES_ENV, raw, DEFAULT_API_MAX_RETRIES)
        return DEFAULT_API_MAX_RETRIES
    return value


def get_api_retry_base_seconds() -> float:
    """Return base delay for transient API retry backoff."""
    raw = os.environ.get(API_RETRY_BASE_SECONDS_ENV)
    if raw is None:
        return DEFAULT_API_RETRY_BASE_SECONDS
    try:
        value = float(raw)
    except ValueError:
        logger.warning(
            "%s=%r is invalid; using %s",
            API_RETRY_BASE_SECONDS_ENV,
            raw,
            DEFAULT_API_RETRY_BASE_SECONDS,
        )
        return DEFAULT_API_RETRY_BASE_SECONDS
    if value < 0:
        logger.warning(
            "%s=%r must be non-negative; using %s",
            API_RETRY_BASE_SECONDS_ENV,
            raw,
            DEFAULT_API_RETRY_BASE_SECONDS,
        )
        return DEFAULT_API_RETRY_BASE_SECONDS
    return value


def build_agent_system_prompt(
    target_name: str,
    descriptor_context: str = "",
) -> str:
    """构建 Agent 模式的 system prompt"""
    return f"""\
你是一个 {target_name} 安全规约生成 Agent。

你的任务：根据给定的参数信息，生成 OracleIR YAML 格式的运行时安全检查规则。

{descriptor_context}

## 你的工作流程

1. 使用 query_index 查找相关的 SpecBlock（参数、消息定义等）
2. 使用 read_block 读取目标参数和关联消息的详细信息
3. 使用 list_specs + read_spec 查看已有的 OracleIR 范例
4. 生成 OracleIR YAML
5. 使用 validate_yaml 校验你的输出
6. 如果校验失败，根据错误信息修改后重新校验
7. 校验通过后，使用 save_spec 保存结果

## 生成规则

1. 使用当前 OracleIR schema，不要使用旧版字段名。
2. id 格式: {{system}}.range.{{name}}，速度/加速度/位置界限使用 type: range_bound。
3. parameters 必须是列表，字段名是 parameters，不是 parameter。
4. observations 必须绑定到真实的 topic 和 field；数组索引用 index 字段，不要写成 velocity[0]。
5. parameter.default 必须与 SpecBlock.structured_fields.default 完全一致。
6. 表达式只能用: + - * / ** sqrt abs min max norm mean degrees acos is_valid param()。
7. 时间导数类约束（acceleration, jerk, rate-of-change）必须使用 window.type: sequential_pairs，并通过 (current - prev_current) / dt 计算；禁止把 velocity 直接和 acceleration/jerk 参数比较。
8. feedback 必须是列表；feedback.direction 表示 fuzz 引导方向，不是安全方向。若 metric 是剩余裕量（如 limit - abs(observed) 或 observed - lower_limit），direction 必须用 minimize；若 metric 是观测压力/比值（如 abs(observed) 或 abs(observed)/limit），direction 才用 maximize。
9. 可执行 spec 优先使用目标 descriptor 中的运行时 topics；如果必须使用未录制或非默认 topic，应在 provenance.evidence 中说明需要扩展 watchlist，不能静默假设 topic 已被记录。
10. provenance.chunk_id 必须是 SpecIndex 中存在的 block_id。
11. validate_yaml 返回 valid: true 后，必须立刻调用 save_spec，不要重复 validate_yaml。

## 当前 schema 示例

```yaml
id: moveit2_panda.range.panda_joint1_max_velocity
type: range_bound
system: moveit2_panda
version: moveit2-2.12.4_panda-3.1.0_jazzy
scope:
  operating_modes: [executing]
observations:
  - name: joint1_velocity
    topic: /joint_states
    field: velocity
    index: 0
    unit: rad/s
parameters:
  - name: joint_limits.panda_joint1.max_velocity
    source: system_doc/moveit2_panda/config/joint_limits.yaml
    unit: rad/s
    default: 2.175
assertions:
  - expr: "abs(joint1_velocity) <= param('joint_limits.panda_joint1.max_velocity') + tolerance"
    tolerance: 0.000001
    severity: error
window:
  type: every_sample
feedback:
  - name: joint1_velocity_margin
    metric: "param('joint_limits.panda_joint1.max_velocity') - abs(joint1_velocity)"
    direction: minimize
provenance:
  - chunk_id: moveit2_panda.parameter.joint_limits_panda_joint1_max_velocity_1
    source_file: system_doc/moveit2_panda/config/joint_limits.yaml
```

## 重要

- 每个参数生成一个独立的 OracleIR spec
- 如果参数不适合生成 oracle（如内部调试参数），直接说明原因并跳过
- 文件名格式: {{system}}_{{category}}_{{param_name_lower}}.yaml
"""


def call_with_transient_api_retries(
    config: APIConfig,
    system_prompt: str,
    messages: list[dict],
    tools: list[dict],
) -> dict:
    """Call provider API with slow retries for transient overload/rate failures."""
    max_retries = get_api_max_retries()
    base_delay = get_api_retry_base_seconds()

    for attempt in range(max_retries + 1):
        try:
            return config.call_with_tools(
                system_prompt=system_prompt,
                messages=messages,
                tools=tools,
            )
        except Exception as exc:
            if not _is_transient_api_error(exc) or attempt >= max_retries:
                raise
            delay = base_delay * (2 ** attempt)
            logger.warning(
                "  API transient failure; retrying in %.1fs (attempt %s/%s): %s",
                delay,
                attempt + 1,
                max_retries,
                _summarize_exception(exc),
            )
            if delay > 0:
                time.sleep(delay)

    raise RuntimeError("unreachable API retry state")


def run_agent_task(
    task_prompt: str,
    config: APIConfig,
    tool_executor: ToolExecutor,
    system_prompt: str,
) -> dict:
    """
    运行单个 Agent 任务的完整对话循环。

    Args:
        task_prompt: 用户任务描述（如"为参数 MPC_XY_VEL_MAX 生成 OracleIR"）
        config: API 配置
        tool_executor: 工具执行器实例
        system_prompt: 系统提示词

    Returns:
        {"success": bool, "message": str, "turns": int}
    """
    messages = [{"role": "user", "content": task_prompt}]

    max_turns = get_max_turns()

    for turn in range(max_turns):
        logger.info(f"  [Turn {turn + 1}] 调用 LLM...")

        response = call_with_transient_api_retries(
            config=config,
            system_prompt=system_prompt,
            messages=messages,
            tools=TOOL_DEFINITIONS,
        )

        stop_reason = response["stop_reason"]
        content_blocks = response["content"]

        # 收集 assistant 回复
        assistant_content = content_blocks

        # 如果模型结束对话（没有 tool_use）
        if stop_reason == "end_turn":
            # 提取最终文本
            final_text = ""
            for block in content_blocks:
                if block["type"] == "text":
                    final_text += block["text"]
            if final_text:
                logger.warning(f"  Agent 结束但未保存 spec (turns={turn + 1})")
                logger.debug(f"  最终回复: {final_text[:200]}")
                return {"success": False, "message": "Agent ended without save_spec", "turns": turn + 1}
            logger.warning(f"  Agent 结束但未保存 spec (turns={turn + 1})")
            return {"success": False, "message": "Agent ended without save_spec", "turns": turn + 1}

        # 处理 tool_use
        messages.append({"role": "assistant", "content": assistant_content})

        tool_results = []
        for block in content_blocks:
            if block["type"] == "tool_use":
                tool_name = block["name"]
                tool_input = block["input"]
                tool_id = block["id"]

                logger.debug(f"    → {tool_name}({_summarize_input(tool_input)})")
                result = tool_executor.execute(tool_name, tool_input)
                logger.debug(f"    ← {result[:200]}")
                if tool_name == "validate_yaml":
                    _log_validation_result(result)
                if tool_name == "save_spec" and _tool_saved_spec(result):
                    logger.info(f"  Agent 保存成功 (turns={turn + 1})")
                    return {"success": True, "message": "saved", "turns": turn + 1}

                tool_results.append({
                    "type": "tool_result",
                    "tool_use_id": tool_id,
                    "content": result,
                })

        messages.append({"role": "user", "content": tool_results})

    # 超过最大轮次
    logger.warning(f"  Agent 超过最大轮次 ({max_turns})")
    return {"success": False, "message": "超过最大对话轮次", "turns": max_turns}


def run_agent_batch(
    block_ids: list[str],
    target_name: str,
    descriptor_context: str,
    blocks_dir: Path,
    index_path: Path,
    specs_dir: Path,
    profile: str | None = None,
) -> dict:
    """
    批量运行 Agent 生成任务。

    Returns:
        {"success": int, "failed": int, "skipped": int, "details": list}
    """
    config = load_config(profile)
    tool_executor = ToolExecutor(blocks_dir, index_path, specs_dir)
    system_prompt = build_agent_system_prompt(target_name, descriptor_context)

    logger.info(f"Agent 模式启动: {len(block_ids)} 个任务")
    logger.info(f"API: {config.provider} / {config.model}")

    results = {"success": 0, "failed": 0, "skipped": 0, "details": []}

    for i, block_id in enumerate(block_ids, 1):
        print(f"\n[{i}/{len(block_ids)}] {block_id}")

        task_prompt = (
            f"为参数 `{block_id}` 生成 OracleIR YAML。\n\n"
            f"步骤：\n"
            f"1. 先用 read_block 读取该参数的详细信息\n"
            f"2. 用 query_index 查找相关的消息定义（by_entity）\n"
            f"3. 用 list_specs 看看已有哪些 spec，选一个相似的用 read_spec 作为参考\n"
            f"4. 生成 YAML 并用 validate_yaml 校验\n"
            f"5. 校验通过后用 save_spec 保存"
        )

        try:
            result = run_agent_task(task_prompt, config, tool_executor, system_prompt)
        except Exception as e:
            logger.error(f"  任务异常: {e}")
            result = {"success": False, "message": f"异常: {e}", "turns": 0}

        results["details"].append({"block_id": block_id, **result})

        if result["success"]:
            results["success"] += 1
            print(f"  ✓ 完成 (turns={result['turns']})")
        else:
            results["failed"] += 1
            print(f"  ✗ 失败: {result['message'][:100]}")

    print(f"\n{'='*50}")
    print(f"Agent 批量生成完成: {results['success']} 成功, {results['failed']} 失败")
    return results


def _summarize_input(inp: dict) -> str:
    """简短摘要工具输入，用于日志"""
    parts = []
    for k, v in inp.items():
        if isinstance(v, str) and len(v) > 50:
            parts.append(f"{k}='{v[:47]}...'")
        else:
            parts.append(f"{k}={v!r}")
    return ", ".join(parts) if parts else ""


def _is_transient_api_error(exc: Exception) -> bool:
    text = str(exc).lower()
    transient_markers = (
        "503",
        "overloaded",
        "rate limit",
        "rate_limit",
        "too many requests",
        "temporarily unavailable",
        "service unavailable",
        "timeout",
        "timed out",
    )
    return any(marker in text for marker in transient_markers)


def _summarize_exception(exc: Exception) -> str:
    text = str(exc).replace("\n", " ")
    if len(text) > 180:
        return text[:177] + "..."
    return text


def _tool_saved_spec(result: str) -> bool:
    try:
        data = json.loads(result)
    except json.JSONDecodeError:
        return False
    return bool(data.get("saved")) and not data.get("error")


def _log_validation_result(result: str) -> None:
    try:
        data = json.loads(result)
    except json.JSONDecodeError:
        logger.warning("    validate_yaml returned non-JSON result")
        return
    if data.get("valid"):
        logger.info("    validate_yaml: valid")
        return
    errors = data.get("errors") or []
    preview = "; ".join(str(error) for error in errors[:3])
    if len(errors) > 3:
        preview += f"; ... and {len(errors) - 3} more"
    logger.warning("    validate_yaml: invalid%s", f" - {preview}" if preview else "")
