#!/usr/bin/env python3
"""
OracleIR 生成 Agent 调用器

这个脚本做一件事：
  把预处理产出的 SpecBlock 喂给 LLM，让它生成 OracleIR YAML。

用法:
    # 为所有速度约束参数生成 oracle
    python -m src.oracle_ir.agent.generate --target px4 --tag velocity_constraint

    # 为单个参数生成 oracle
    python -m src.oracle_ir.agent.generate --target px4 --block-id px4.parameter.MPC_XY_VEL_MAX

    # 干跑模式（只看 prompt，不调 API）
    python -m src.oracle_ir.agent.generate --target px4 --block-id px4.parameter.MPC_XY_VEL_MAX --dry-run
"""

import json
import re
import argparse
import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

# ============================================================
# 第一步：加载预处理产出
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent.parent


def load_blocks(blocks_dir: Path) -> dict[str, dict]:
    """加载所有 SpecBlock JSON 文件，返回 {block_id: block_dict}"""
    blocks = {}
    for f in blocks_dir.glob("*.json"):
        data = json.loads(f.read_text(encoding="utf-8"))
        blocks[data["block_id"]] = data
    return blocks


def load_index(index_path: Path) -> dict:
    """加载 SpecIndex"""
    return json.loads(index_path.read_text(encoding="utf-8"))


# ============================================================
# 第二步：为 Agent 组装上下文（这是最关键的部分）
# ============================================================

class AgentContextBuilder:
    """
    这个类的作用：从几百个 SpecBlock 中，挑出 Agent 真正需要的那几个。

    比如 Agent 要为 MPC_XY_VEL_MAX 生成 oracle，它需要知道：
    1. MPC_XY_VEL_MAX 本身的信息（default=12, unit=m/s）
    2. 速度数据从哪个 topic 的哪个 field 读取
    3. 已有的类似 oracle 长什么样（模仿格式）
    """

    def __init__(self, blocks: dict[str, dict], index: dict, specs_dir: Path):
        self.blocks = blocks
        self.index = index
        self.specs_dir = specs_dir
        # 所有已有 spec 目录（用于回退查找范例）
        self._all_spec_dirs = [d for d in specs_dir.parent.iterdir() if d.is_dir()]

    def build(self, target_block_id: str) -> dict:
        """
        输入: 一个 block_id，如 "px4.parameter.MPC_XY_VEL_MAX"
        输出: Agent 需要的所有上下文信息
        """
        target = self.blocks.get(target_block_id)
        if not target:
            raise ValueError(f"Block not found: {target_block_id}")

        # --- 找关联的消息/字段定义 ---
        # 为什么需要？因为 Agent 要知道"速度"从哪个 ROS topic 读取
        field_blocks = self._find_related_fields(target)

        # --- 找同类已有 oracle ---
        # 为什么需要？给 Agent 一个模仿的范例
        example_spec = self._find_similar_spec(target)

        # --- 找关联的其他参数 ---
        # 为什么需要？有些 oracle 需要引用多个参数
        related_params = self._find_related_params(target)

        return {
            "target": target,
            "field_blocks": field_blocks,
            "example_spec": example_spec,
            "related_params": related_params,
        }

    def _find_related_fields(self, target: dict) -> list[dict]:
        """
        通过 reference_graph 找到与目标参数相关的消息/字段定义。

        例如 MPC_XY_VEL_MAX 的 references 里有 "VehicleLocalPosition"，
        我们就去索引里找 VehicleLocalPosition 对应的 block，
        这样 Agent 就知道速度数据在 /VehicleLocalPosition_PubSubTopic 的 vx/vy 字段。
        """
        results = []
        refs = target.get("references", [])
        entity_index = self.index.get("entity_index", {})

        for ref_name in refs:
            # 在索引中查找这个实体名对应的 block_ids
            block_ids = entity_index.get(ref_name, [])
            for bid in block_ids:
                block = self.blocks.get(bid)
                if block and block.get("block_type") in ("message", "field"):
                    # 只保留关键信息，不要塞太多
                    results.append({
                        "block_id": block["block_id"],
                        "name": block["name"],
                        "block_type": block["block_type"],
                        "structured_fields": block.get("structured_fields", {}),
                    })
        return results[:10]  # 最多 10 个，避免撑爆上下文

    def _find_similar_spec(self, target: dict) -> str | None:
        """
        找一个已有的、类型相似的 OracleIR YAML 作为范例。

        为什么？LLM 看一个具体例子比看 10 页规则文档更有效。
        如果当前目标的 specs 目录为空，会回退到其他目标的 specs 中查找。
        """
        tags = target.get("tags", [])

        # 收集候选目录：当前目标优先，然后回退到其他目标
        candidate_dirs = [self.specs_dir] + [
            d for d in self._all_spec_dirs if d != self.specs_dir
        ]

        for spec_dir in candidate_dirs:
            if not spec_dir.exists():
                continue
            all_specs = sorted(spec_dir.glob("*.yaml"))
            if not all_specs:
                continue

            # 尝试按 tag 关键词匹配文件名
            for tag in tags:
                keyword = tag.replace("_constraint", "").replace("_", "")
                for spec_path in all_specs:
                    if keyword in spec_path.stem.lower():
                        return spec_path.read_text(encoding="utf-8")

            # 兜底：用目录中第一个 spec 作为范例
            return all_specs[0].read_text(encoding="utf-8")

        return None

    def _find_related_params(self, target: dict) -> list[dict]:
        """找同组的其他参数（可能在同一个 oracle 中被引用）"""
        results = []
        refs = target.get("references", [])
        entity_index = self.index.get("entity_index", {})

        for ref_name in refs:
            block_ids = entity_index.get(ref_name, [])
            for bid in block_ids:
                block = self.blocks.get(bid)
                if block and block.get("block_type") == "parameter":
                    results.append({
                        "name": block["name"],
                        "structured_fields": block.get("structured_fields", {}),
                    })
        return results[:5]


# ============================================================
# 第三步：把上下文拼成 Prompt（Agent 看到的就是这个）
# ============================================================

def build_system_prompt(target_name: str, descriptor_context: str = "") -> str:
    """根据目标系统动态生成 system prompt"""
    return f"""\
你是一个 {target_name} 安全规约专家。你的任务是根据给定的参数规格，
生成 OracleIR YAML 格式的运行时检查规则。

{descriptor_context}

生成规则：
1. id 格式: {{system}}.{{category}}.{{name}}，category 从 range/validity/consistency 中选
2. observations 必须绑定到真实的 topic 和 field（从下文给出的可用 topics 中选择）
3. parameter.default 必须与给定的 structured_fields.default 完全一致
4. 表达式只能用: + - * / ** sqrt abs min max norm mean degrees acos is_valid
5. feedback.direction: 如果是上限约束用 maximize，下限约束用 minimize
6. provenance.chunk_id 必须填写给定的 block_id

只输出 YAML，不要解释。
"""


def build_prompt(context: dict) -> str:
    """
    把上下文拼成一个完整的 prompt。

    这就是 Agent 实际看到的内容。
    """
    target = context["target"]
    parts = []

    # --- 告诉 Agent 要为哪个参数生成 oracle ---
    parts.append("## 目标参数\n")
    parts.append(f"block_id: {target['block_id']}")
    parts.append(f"name: {target['name']}")
    parts.append(f"type: {target['block_type']}")
    sf = target.get("structured_fields", {})
    parts.append(f"structured_fields: {json.dumps(sf, ensure_ascii=False)}")
    parts.append(f"tags: {target.get('tags', [])}")
    parts.append(f"natural_language: {target.get('natural_language', '')}")
    parts.append("")

    # --- 告诉 Agent 数据从哪里读 ---
    if context["field_blocks"]:
        parts.append("## 可用的消息/字段定义（用于确定 observation 的 topic 和 field）\n")
        for fb in context["field_blocks"]:
            parts.append(f"- {fb['name']} ({fb['block_type']})")
            parts.append(f"  block_id: {fb['block_id']}")
            sf = fb.get("structured_fields", {})
            if sf:
                parts.append(f"  fields: {json.dumps(sf, ensure_ascii=False)}")
        parts.append("")

    # --- 给 Agent 一个范例 ---
    if context["example_spec"]:
        parts.append("## 参考范例（模仿这个格式）\n")
        parts.append("```yaml")
        parts.append(context["example_spec"].strip())
        parts.append("```")
        parts.append("")

    # --- 关联参数 ---
    if context["related_params"]:
        parts.append("## 关联参数（可能需要在同一个 oracle 中引用）\n")
        for rp in context["related_params"]:
            parts.append(f"- {rp['name']}: {json.dumps(rp['structured_fields'], ensure_ascii=False)}")
        parts.append("")

    # --- 最终指令 ---
    parts.append("## 请生成\n")
    parts.append(f"为参数 `{target['name']}` 生成一个 OracleIR YAML。")
    parts.append("直接输出 YAML 内容，不要包含 ```yaml 标记。")
    parts.append("")
    parts.append("## 必需的顶层字段\n")
    parts.append("id, type, system, observations, assertions, provenance")
    parts.append("其中 id 和 type 是必填项，缺一不可。")

    return "\n".join(parts)


# ============================================================
# 第四步：调用 LLM Agent
# ============================================================

def call_agent(system_prompt: str, user_prompt: str, dry_run: bool = False) -> str:
    """
    调用 LLM 生成 YAML。

    通过 api_config.yaml 管理 API 配置，支持 Anthropic / OpenAI 兼容接口。
    """
    if dry_run:
        print("=" * 60)
        print("DRY RUN — 以下是发给 Agent 的 prompt:")
        print("=" * 60)
        print(f"\n[System]\n{system_prompt}\n")
        print(f"[User]\n{user_prompt}\n")
        print("=" * 60)
        return ""

    from src.oracle_ir.agent.api_manager import load_config
    config = load_config()
    logger.info(f"使用 API: {config.provider} / {config.model}")
    return config.call(system_prompt, user_prompt)


# ============================================================
# 第五步：校验 Agent 的输出
# ============================================================

def strip_yaml_fences(text: str) -> str:
    """剥离 LLM 输出中常见的 markdown 代码围栏"""
    text = text.strip()
    m = re.match(r'^```(?:yaml|yml)?\s*\n(.*?)```\s*$', text, re.DOTALL)
    if m:
        return m.group(1)
    return text


def validate_output(yaml_text: str) -> tuple[bool, list[str]]:
    """
    用已有的 Validator 检查 Agent 生成的 YAML 是否合法。
    返回 (是否通过, 错误列表)
    """
    import yaml as yaml_lib
    from src.oracle_ir.transform.parser import load_oracle_ir
    from src.oracle_ir.transform.validator import validate_oracle_ir

    yaml_text = strip_yaml_fences(yaml_text)

    # 先检查 YAML 语法
    try:
        data = yaml_lib.safe_load(yaml_text)
    except yaml_lib.YAMLError as e:
        return False, [f"YAML 语法错误: {e}"]

    if not isinstance(data, dict):
        return False, ["输出不是有效的 YAML 字典"]

    # 用 OracleIR 的 validator 做完整校验
    try:
        from src.oracle_ir.transform.parser import _dict_to_oracle_ir
        ir = _dict_to_oracle_ir(data)
        result = validate_oracle_ir(ir)
        return result.valid, result.errors
    except Exception as e:
        return False, [f"解析失败: {e}"]


# ============================================================
# 第六步：如果校验失败，让 Agent 修复
# ============================================================

def fix_with_agent(yaml_text: str, errors: list[str], system_prompt: str, dry_run: bool = False) -> str:
    """把错误信息反馈给 Agent，让它修复"""
    fix_prompt = (
        "你之前生成的 OracleIR YAML 校验失败了。\n\n"
        f"错误信息:\n" + "\n".join(f"  - {e}" for e in errors) + "\n\n"
        f"原始输出:\n```yaml\n{yaml_text}\n```\n\n"
        "请修复以上错误，重新输出完整的 YAML。只输出 YAML，不要解释。"
    )
    return call_agent(system_prompt, fix_prompt, dry_run=dry_run)


# ============================================================
# 主流程：把上面所有步骤串起来
# ============================================================

def generate_oracle(
    block_id: str,
    blocks: dict[str, dict],
    index: dict,
    specs_dir: Path,
    output_dir: Path,
    target_name: str = "",
    descriptor_context: str = "",
    dry_run: bool = False,
    max_retries: int = 2,
) -> Path | None:
    """
    完整流程：组装上下文 → 调用 Agent → 校验 → 保存

    Returns:
        生成的 YAML 文件路径，失败返回 None
    """
    # 1. 组装上下文
    builder = AgentContextBuilder(blocks, index, specs_dir)
    context = builder.build(block_id)

    # 2. 生成 prompt
    system_prompt = build_system_prompt(target_name, descriptor_context)
    prompt = build_prompt(context)

    # 3. 调用 Agent
    yaml_text = call_agent(system_prompt, prompt, dry_run=dry_run)
    if dry_run:
        return None

    # 4. 校验 + 修复循环
    for attempt in range(max_retries + 1):
        valid, errors = validate_output(yaml_text)
        if valid:
            break
        if attempt < max_retries:
            logger.warning(f"校验失败 (尝试 {attempt + 1})，让 Agent 修复...")
            logger.warning(f"  错误: {errors}")
            yaml_text = fix_with_agent(yaml_text, errors, system_prompt)
        else:
            logger.error(f"校验失败，已达最大重试次数。错误: {errors}")
            return None

    # 5. 保存结果（确保去除围栏）
    yaml_text = strip_yaml_fences(yaml_text)
    output_dir.mkdir(parents=True, exist_ok=True)
    # 文件名从 block_id 生成
    safe_name = block_id.replace(".", "_").replace("/", "_")
    output_path = output_dir / f"{safe_name}.yaml"
    output_path.write_text(yaml_text, encoding="utf-8")
    logger.info(f"已保存: {output_path}")
    return output_path


# ============================================================
# CLI 入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="用 LLM Agent 生成 OracleIR YAML")
    parser.add_argument("--target", required=True, help="目标系统名 (px4, turtlebot3, ...)")
    parser.add_argument("--block-id", help="指定单个 block_id")
    parser.add_argument("--tag", help="按语义标签批量生成 (如 velocity_constraint)")
    parser.add_argument("--all", action="store_true", help="提取所有参数类型的 block")
    parser.add_argument("--mode", choices=["simple", "agent"], default="agent",
                        help="生成模式: simple=单次调用, agent=tool_use自主循环 (默认 agent)")
    parser.add_argument("--dry-run", action="store_true", help="只打印 prompt，不调 API")
    parser.add_argument("--output", default=None, help="输出目录")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s: %(message)s",
    )

    target = args.target

    # 加载数据（路径由 target 决定）
    blocks_dir = PROJECT_ROOT / "system_doc" / "preprocessed" / target / "blocks"
    index_path = PROJECT_ROOT / "system_doc" / "preprocessed" / target / "index.json"
    specs_dir = PROJECT_ROOT / "src" / "oracle_ir" / "specs" / target
    output_dir = Path(args.output) if args.output else specs_dir

    if not blocks_dir.exists():
        print(f"错误: 预处理产出不存在。请先运行:")
        print(f"  python -m system_doc.preprocessing.run_pipeline --target {target}")
        return

    # 加载 descriptor（可选）
    descriptor_context = ""
    descriptor_path = PROJECT_ROOT / "src" / "oracle_ir" / "targets" / f"{target}.yaml"
    if descriptor_path.exists():
        from src.oracle_ir.targets.descriptor import TargetDescriptor
        descriptor = TargetDescriptor.load(descriptor_path)
        descriptor_context = descriptor.to_agent_context()
        target_display = descriptor.display_name
    else:
        target_display = target

    blocks = load_blocks(blocks_dir)
    index = json.loads(index_path.read_text(encoding="utf-8"))
    print(f"目标: {target_display}")
    print(f"已加载 {len(blocks)} 个 SpecBlock")
    print(f"模式: {args.mode}")

    # 确定要处理的 block_ids
    if args.block_id:
        target_ids = [args.block_id]
    elif args.tag:
        tag_index = index.get("tag_index", {})
        all_ids = tag_index.get(args.tag, [])
        target_ids = [bid for bid in all_ids
                      if blocks.get(bid, {}).get("block_type") == "parameter"]
        print(f"标签 '{args.tag}' 下有 {len(target_ids)} 个参数待生成")
    elif getattr(args, 'all'):
        target_ids = [bid for bid, b in blocks.items()
                      if b.get("block_type") == "parameter"]
        print(f"全部参数: {len(target_ids)} 个待生成")
    else:
        print("请指定 --block-id、--tag 或 --all")
        return

    # ─── Agent 模式 ───
    if args.mode == "agent" and not args.dry_run:
        from src.oracle_ir.agent.agent_loop import run_agent_batch
        run_agent_batch(
            block_ids=target_ids,
            target_name=target_display,
            descriptor_context=descriptor_context,
            blocks_dir=blocks_dir,
            index_path=index_path,
            specs_dir=output_dir,
        )
        return

    # ─── Simple 模式（原有逻辑） ───
    success, fail = 0, 0
    for bid in target_ids:
        print(f"\n--- 生成: {bid} ---")
        result = generate_oracle(
            block_id=bid,
            blocks=blocks,
            index=index,
            specs_dir=specs_dir,
            output_dir=output_dir,
            target_name=target_display,
            descriptor_context=descriptor_context,
            dry_run=args.dry_run,
        )
        if result:
            success += 1
        else:
            fail += 1

    if not args.dry_run:
        print(f"\n完成: {success} 成功, {fail} 失败")


if __name__ == "__main__":
    main()
