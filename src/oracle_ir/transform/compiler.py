"""OracleIR 编译器 — 将 OracleIR 编译为可执行 oracle"""

from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import Any, Callable
from pathlib import Path

from .schema import OracleIR, Assertion, FeedbackSpec
from .expr_engine import compile_expr


@dataclass
class CompiledOracle:
    """编译后的可执行 oracle"""
    id: str
    system: str
    oracle_type: str

    # 编译产物
    _check_fn: Callable | None = field(default=None, repr=False)
    _feedback_updaters: list[dict] = field(default_factory=list, repr=False)
    _ir: OracleIR | None = field(default=None, repr=False)

    def check(self, config, msg_list, state_dict, feedback_list=None):
        """执行 oracle 检查，返回错误列表"""
        if self._check_fn is None:
            return []
        return self._check_fn(config, msg_list, state_dict, feedback_list)


def compile_oracle_ir(ir: OracleIR) -> CompiledOracle:
    """将 OracleIR 编译为 CompiledOracle"""
    compiler = _OracleCompiler(ir)
    return compiler.compile()


class _OracleCompiler:
    """内部编译器实现"""

    def __init__(self, ir: OracleIR):
        self.ir = ir
        self._derived_fns: dict[str, Callable] = {}
        self._assertion_fns: list[tuple[Callable, Assertion]] = []
        self._feedback_fns: list[tuple[Callable, FeedbackSpec]] = []

    def compile(self) -> CompiledOracle:
        # 编译派生量表达式
        var_names = self._collect_var_names()
        for d in self.ir.derived:
            self._derived_fns[d.name] = compile_expr(d.expr, var_names)

        # 编译断言表达式
        for assertion in self.ir.assertions:
            fn = compile_expr(assertion.expr, var_names)
            self._assertion_fns.append((fn, assertion))

        # 编译 feedback 表达式
        for fb in self.ir.feedback:
            fn = compile_expr(fb.metric, var_names)
            self._feedback_fns.append((fn, fb))

        # 构建检查函数
        check_fn = self._build_check_fn()

        return CompiledOracle(
            id=self.ir.id,
            system=self.ir.system,
            oracle_type=self.ir.type,
            _check_fn=check_fn,
            _ir=self.ir,
        )

    def _collect_var_names(self) -> list[str]:
        names = []
        for obs in self.ir.observations:
            names.append(obs.name)
        for p in self.ir.parameters:
            names.append(p.name)
        for c in self.ir.constants:
            names.append(c.name)
        for d in self.ir.derived:
            names.append(d.name)
        names.append("tolerance")
        return names

    def _build_check_fn(self) -> Callable:
        """构建最终的 check 函数闭包"""
        ir = self.ir
        derived_fns = self._derived_fns
        assertion_fns = self._assertion_fns
        feedback_fns = self._feedback_fns

        def check_fn(config, msg_list, state_dict, feedback_list=None):
            errs = []

            # 提取观测数据序列
            topic_data = {}
            for obs in ir.observations:
                topic_data[obs.topic] = state_dict.get(obs.topic, [])

            # 确定采样点（以第一个 observation 的 topic 为基准）
            if not ir.observations:
                return errs
            primary_topic = ir.observations[0].topic
            samples = topic_data.get(primary_topic, [])
            if not samples:
                return errs

            # 地面过滤时间戳
            ground_ts = set()
            if ir.scope.require_airborne:
                ground_ts = _get_ground_timestamps(state_dict)

            # 参数解析器
            param_defaults = {p.name: p.default for p in ir.parameters if p.default is not None}

            def param_resolver(name):
                # 动态阈值：如果正在变异该参数，使用变异值
                if hasattr(config, 'exp_pgfuzz') and config.exp_pgfuzz and msg_list:
                    param_msg = msg_list[0]
                    if (hasattr(param_msg, 'param_name') and
                            param_msg.param_name == name):
                        return param_msg.value
                return param_defaults.get(name, 0.0)

            # 逐采样点检查
            for ts, msg in samples:
                # scope 过滤
                if ir.scope.require_airborne and ts in ground_ts:
                    continue
                if ir.scope.flight_modes:
                    if (hasattr(config, 'flight_mode') and
                            config.flight_mode not in ir.scope.flight_modes):
                        continue

                # 构建变量上下文
                ctx = _build_context(
                    ts, ir, state_dict, topic_data, param_resolver
                )
                if ctx is None:
                    continue

                # 计算派生量
                for name, fn in derived_fns.items():
                    val = fn(ctx)
                    if val is None:
                        break
                    ctx[name] = val
                else:
                    # 检查断言
                    for assert_fn, assertion in assertion_fns:
                        ctx["tolerance"] = assertion.tolerance
                        result = assert_fn(ctx)
                        if result is False:
                            msg_text = assertion.message.format(**ctx) if assertion.message else f"{ir.id} violated"
                            errs.append(f"{ts} {msg_text}")

                    # 更新 feedback
                    if feedback_list is not None:
                        for fb_fn, fb_spec in feedback_fns:
                            val = fb_fn(ctx)
                            if val is not None:
                                _update_feedback(
                                    feedback_list, fb_spec, val
                                )

            return errs

        return check_fn


# =========================================================================
# Helper functions
# =========================================================================

def _build_context(ts, ir, state_dict, topic_data, param_resolver):
    """为单个采样点构建变量上下文"""
    ctx = {"_param_resolver": param_resolver, "_ts": ts,
           "_param_names": [p.name for p in ir.parameters]}

    # 绑定观测变量
    for obs in ir.observations:
        samples = topic_data.get(obs.topic, [])
        # 找到时间最接近的采样
        val = _get_field_at_ts(samples, ts, obs.field, obs.index)
        if val is None:
            return None
        if not (math.isfinite(val) if isinstance(val, float) else True):
            return None
        ctx[obs.name] = val

    # 绑定参数
    for p in ir.parameters:
        ctx[p.name] = param_resolver(p.name)

    # 绑定常量
    for c in ir.constants:
        ctx[c.name] = c.value

    return ctx


def _get_field_at_ts(samples, target_ts, field_name, index=None):
    """从采样序列中获取指定时间戳最近的字段值"""
    if not samples:
        return None

    # 找时间最近的采样
    best_msg = None
    best_diff = float("inf")
    for ts, msg in samples:
        diff = abs(ts - target_ts)
        if diff < best_diff:
            best_diff = diff
            best_msg = msg

    if best_msg is None:
        return None

    # 获取字段值
    val = getattr(best_msg, field_name, None)
    if val is None:
        return None
    if index is not None:
        try:
            val = val[index]
        except (IndexError, TypeError):
            return None
    return float(val) if val is not None else None


def _get_ground_timestamps(state_dict, ground_dist=0.15):
    """获取地面接触的时间戳集合"""
    ground_ts = set()
    pos_data = state_dict.get("/VehicleLocalPosition_PubSubTopic", [])
    for ts, pos in pos_data:
        if hasattr(pos, "dist_bottom") and pos.dist_bottom < ground_dist:
            ground_ts.add(ts)
    return ground_ts


def _update_feedback(feedback_list, fb_spec, value):
    """更新 feedback 列表中对应的 feedback 实例"""
    for fb in feedback_list:
        if fb.name == fb_spec.name:
            fb.update_value(value)
            return


def load_compiled_oracles(target: str, spec_dir: Path | None = None):
    """加载并编译指定目标的所有 OracleIR spec"""
    from .parser import load_all_specs

    if spec_dir is None:
        spec_dir = Path(__file__).parent / "specs" / target

    if not spec_dir.exists():
        return []

    specs = load_all_specs(spec_dir)
    return [compile_oracle_ir(ir) for ir in specs]


