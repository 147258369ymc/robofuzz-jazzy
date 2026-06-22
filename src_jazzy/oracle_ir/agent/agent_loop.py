"""Agent Loop — 基于 tool_use 的自主 OracleIR 生成循环

与 simple 模式的区别：
  - Agent 自己决定查询哪些 block、读取哪些参考
  - Agent 自己调用 validator 检查输出
  - Agent 根据错误自主修复，无需硬编码重试逻辑
  - 支持批量生成时的上下文积累（已生成的 spec 可被后续任务参考）
"""

from __future__ import annotations
import logging
from pathlib import Path
from typing import Any

from .api_manager import load_config, APIConfig
from .tools import TOOL_DEFINITIONS, ToolExecutor

logger = logging.getLogger(__name__)

MAX_TURNS = 25  # 单个任务最大对话轮次，防止无限循环


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

1. id 格式: {{system}}.{{category}}.{{name}}，category 从 range/validity/consistency 中选
2. observations 必须绑定到真实的 topic 和 field
3. parameter.default 必须与 SpecBlock.structured_fields.default 完全一致
4. 表达式只能用: + - * / ** sqrt abs min max norm mean degrees acos is_valid param()
5. feedback.direction: 上限约束用 maximize，下限约束用 minimize
6. provenance.chunk_id 必须是 SpecIndex 中存在的 block_id

## 重要

- 每个参数生成一个独立的 OracleIR spec
- 如果参数不适合生成 oracle（如内部调试参数），直接说明原因并跳过
- 文件名格式: {{system}}_{{category}}_{{param_name_lower}}.yaml
"""


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

    for turn in range(MAX_TURNS):
        logger.info(f"  [Turn {turn + 1}] 调用 LLM...")

        response = config.call_with_tools(
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
            logger.info(f"  Agent 完成 (turns={turn + 1})")
            if final_text:
                logger.debug(f"  最终回复: {final_text[:200]}")
            return {"success": True, "message": final_text, "turns": turn + 1}

        # 处理 tool_use
        messages.append({"role": "assistant", "content": assistant_content})

        tool_results = []
        for block in content_blocks:
            if block["type"] == "tool_use":
                tool_name = block["name"]
                tool_input = block["input"]
                tool_id = block["id"]

                logger.info(f"    → {tool_name}({_summarize_input(tool_input)})")
                result = tool_executor.execute(tool_name, tool_input)
                logger.debug(f"    ← {result[:200]}")

                tool_results.append({
                    "type": "tool_result",
                    "tool_use_id": tool_id,
                    "content": result,
                })

        messages.append({"role": "user", "content": tool_results})

    # 超过最大轮次
    logger.warning(f"  Agent 超过最大轮次 ({MAX_TURNS})")
    return {"success": False, "message": "超过最大对话轮次", "turns": MAX_TURNS}


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
