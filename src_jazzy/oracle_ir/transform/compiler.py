"""OracleIR 编译器 — 将 OracleIR 编译为可执行 oracle"""

from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import Any, Callable
from pathlib import Path

from ..schema import OracleIR, Assertion, FeedbackSpec
from .expr_engine import compile_expr


# 默认参数解析器：仅使用 parameter.default
def _default_param_resolver_factory(ir: OracleIR):
    """返回一个只查 default 值的 param_resolver"""
    defaults = {p.name: p.default for p in ir.parameters if p.default is not None}

    def resolver(name: str) -> float:
        return defaults.get(name, 0.0)

    return resolver


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


def compile_oracle_ir(
    ir: OracleIR,
    param_resolver: Callable[[str], float] | None = None,
) -> CompiledOracle:
    """
    将 OracleIR 编译为 CompiledOracle。

    Args:
        ir: OracleIR 实例
        param_resolver: 外部参数解析器 (name → value)。
                        若为 None，使用 parameter.default 作为默认值。
    """
    compiler = _OracleCompiler(ir, param_resolver)
    return compiler.compile()


class _OracleCompiler:
    """内部编译器实现"""

    def __init__(self, ir: OracleIR, param_resolver: Callable | None = None):
        self.ir = ir
        self._param_resolver = param_resolver or _default_param_resolver_factory(ir)
        self._derived_fns: dict[str, Callable] = {}
        self._assertion_fns: list[tuple[Callable, Assertion]] = []
        self._feedback_fns: list[tuple[Callable, FeedbackSpec]] = []
        self._scope_filter_fn: Callable | None = None

    def compile(self) -> CompiledOracle:
        # 编译 scope filter 表达式
        if self.ir.scope.filter_expr:
            filter_vars = [obs.name for obs in self.ir.scope.filter_observations]
            self._scope_filter_fn = compile_expr(self.ir.scope.filter_expr, filter_vars)

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
        """构建最终的 check 函数闭包，根据 window.type 分发"""
        window_type = self.ir.window.type

        if window_type == "sequential_pairs":
            return self._build_sequential_pairs_fn()
        elif window_type == "aggregation":
            return self._build_aggregation_fn()
        else:
            # every_sample (default)
            return self._build_every_sample_fn()

    def _build_every_sample_fn(self) -> Callable:
        """逐采样点检查（默认模式）"""
        ir = self.ir
        derived_fns = self._derived_fns
        assertion_fns = self._assertion_fns
        feedback_fns = self._feedback_fns
        param_resolver = self._param_resolver
        scope_filter_fn = self._scope_filter_fn

        def check_fn(config, msg_list, state_dict, feedback_list=None):
            errs = []

            # 提取观测数据序列
            topic_data = {}
            for obs in ir.observations:
                topic_data[obs.topic] = state_dict.get(obs.topic, [])

            if not ir.observations:
                return errs
            primary_topic = ir.observations[0].topic
            samples = topic_data.get(primary_topic, [])
            if not samples:
                return errs

            # scope filter 数据（来自 filter_observations）
            filter_topic_data = {}
            for obs in ir.scope.filter_observations:
                filter_topic_data[obs.topic] = state_dict.get(obs.topic, [])

            # 逐采样点检查
            for ts, msg in samples:
                # scope: flight_modes 检查
                if ir.scope.flight_modes:
                    if (hasattr(config, 'flight_mode') and
                            config.flight_mode not in ir.scope.flight_modes):
                        continue

                # scope: filter_expr 检查
                if scope_filter_fn and not _eval_scope_filter(
                    scope_filter_fn, ts, ir.scope.filter_observations,
                    filter_topic_data
                ):
                    continue

                ctx = _build_context(ts, ir, state_dict, topic_data, param_resolver)
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
                            msg_text = (assertion.message.format(**ctx)
                                        if assertion.message else f"{ir.id} violated")
                            errs.append(f"{ts} {msg_text}")

                    # 更新 feedback
                    if feedback_list is not None:
                        for fb_fn, fb_spec in feedback_fns:
                            val = fb_fn(ctx)
                            if val is not None:
                                _update_feedback(feedback_list, fb_spec, val)

            return errs

        return check_fn

    def _build_sequential_pairs_fn(self) -> Callable:
        """连续采样对检查 — 用于时序一致性（导数、积分、配对比较）"""
        ir = self.ir
        derived_fns = self._derived_fns
        assertion_fns = self._assertion_fns
        feedback_fns = self._feedback_fns
        param_resolver = self._param_resolver
        scope_filter_fn = self._scope_filter_fn

        def check_fn(config, msg_list, state_dict, feedback_list=None):
            errs = []
            topic_data = {}
            for obs in ir.observations:
                topic_data[obs.topic] = state_dict.get(obs.topic, [])

            if not ir.observations:
                return errs
            primary_topic = ir.observations[0].topic
            samples = topic_data.get(primary_topic, [])
            if len(samples) < 2:
                return errs

            filter_topic_data = {}
            for obs in ir.scope.filter_observations:
                filter_topic_data[obs.topic] = state_dict.get(obs.topic, [])

            max_metric_vals = {}

            for i in range(1, len(samples)):
                ts_prev, msg_prev = samples[i - 1]
                ts_curr, msg_curr = samples[i]

                if ir.scope.flight_modes:
                    if (hasattr(config, 'flight_mode') and
                            config.flight_mode not in ir.scope.flight_modes):
                        continue

                if scope_filter_fn and not _eval_scope_filter(
                    scope_filter_fn, ts_curr, ir.scope.filter_observations,
                    filter_topic_data
                ):
                    continue

                dt = (ts_curr - ts_prev) / 1e9
                if dt <= 0 or dt > 1.0:
                    continue

                ctx_prev = _build_context(ts_prev, ir, state_dict, topic_data, param_resolver)
                ctx_curr = _build_context(ts_curr, ir, state_dict, topic_data, param_resolver)
                if ctx_prev is None or ctx_curr is None:
                    continue

                # 合并为 pair 上下文
                ctx = dict(ctx_curr)
                for key, val in ctx_prev.items():
                    if not key.startswith("_"):
                        ctx[f"prev_{key}"] = val
                ctx["dt"] = dt
                ctx["ts_curr"] = ts_curr

                # 计算派生量
                valid = True
                for name, fn in derived_fns.items():
                    val = fn(ctx)
                    if val is None:
                        valid = False
                        break
                    ctx[name] = val

                if not valid:
                    continue

                # 检查断言
                for assert_fn, assertion in assertion_fns:
                    ctx["tolerance"] = assertion.tolerance
                    result = assert_fn(ctx)
                    if result is False:
                        msg_text = (assertion.message.format(**ctx)
                                    if assertion.message else f"{ir.id} violated")
                        errs.append(f"{ts_curr} {msg_text}")

                # 更新 feedback (取最大值)
                if feedback_list is not None:
                    for fb_fn, fb_spec in feedback_fns:
                        val = fb_fn(ctx)
                        if val is not None:
                            prev = max_metric_vals.get(fb_spec.name, None)
                            if prev is None or val > prev:
                                max_metric_vals[fb_spec.name] = val

            # feedback 最终更新
            if feedback_list is not None:
                for fb_spec_name, val in max_metric_vals.items():
                    for fb in feedback_list:
                        if fb.name == fb_spec_name:
                            fb.update_value(val)

            return errs

        return check_fn

    def _build_aggregation_fn(self) -> Callable:
        """聚合统计检查 — 用于位置保持、传感器一致性统计"""
        ir = self.ir
        derived_fns = self._derived_fns
        assertion_fns = self._assertion_fns
        feedback_fns = self._feedback_fns
        param_resolver = self._param_resolver

        def check_fn(config, msg_list, state_dict, feedback_list=None):
            errs = []

            # 作用域检查
            if ir.scope.flight_modes:
                if (hasattr(config, 'flight_mode') and
                        config.flight_mode not in ir.scope.flight_modes):
                    return errs

            topic_data = {}
            for obs in ir.observations:
                topic_data[obs.topic] = state_dict.get(obs.topic, [])

            if not ir.observations:
                return errs
            primary_topic = ir.observations[0].topic
            samples = topic_data.get(primary_topic, [])
            if not samples:
                return errs

            # 收集所有观测值序列
            series = {obs.name: [] for obs in ir.observations}
            for ts, msg in samples:
                for obs in ir.observations:
                    val = _get_field_value(msg, obs.field, obs.index)
                    if val is not None:
                        series[obs.name].append(val)

            # 构建聚合上下文
            ctx = {"_param_resolver": param_resolver, "_ts": 0,
                   "_param_names": [p.name for p in ir.parameters]}
            for p in ir.parameters:
                ctx[p.name] = param_resolver(p.name)
            for c in ir.constants:
                ctx[c.name] = c.value

            # 将序列统计量放入上下文
            import statistics as stat_mod
            for name, vals in series.items():
                if vals:
                    ctx[name] = vals
                    ctx[f"{name}_max"] = max(vals)
                    ctx[f"{name}_min"] = min(vals)
                    ctx[f"{name}_mean"] = stat_mod.mean(vals)
                    ctx[f"{name}_std"] = stat_mod.stdev(vals) if len(vals) > 1 else 0.0
                    ctx[f"{name}_count"] = len(vals)
                else:
                    ctx[name] = []
                    ctx[f"{name}_max"] = 0.0
                    ctx[f"{name}_min"] = 0.0
                    ctx[f"{name}_mean"] = 0.0
                    ctx[f"{name}_std"] = 0.0
                    ctx[f"{name}_count"] = 0

            # 计算派生量
            valid = True
            for name, fn in derived_fns.items():
                val = fn(ctx)
                if val is None:
                    valid = False
                    break
                ctx[name] = val

            if not valid:
                return errs

            # 检查断言
            for assert_fn, assertion in assertion_fns:
                ctx["tolerance"] = assertion.tolerance
                result = assert_fn(ctx)
                if result is False:
                    msg_text = (assertion.message.format(**ctx)
                                if assertion.message else f"{ir.id} violated")
                    errs.append(msg_text)

            # 更新 feedback
            if feedback_list is not None:
                for fb_fn, fb_spec in feedback_fns:
                    val = fb_fn(ctx)
                    if val is not None:
                        _update_feedback(feedback_list, fb_spec, val)

            return errs

        return check_fn


# =========================================================================
# Helper functions
# =========================================================================

def _eval_scope_filter(filter_fn, ts, filter_observations, filter_topic_data):
    """评估 scope filter 表达式，返回 True 表示采样点通过过滤"""
    ctx = {}
    for obs in filter_observations:
        samples = filter_topic_data.get(obs.topic, [])
        val = _get_field_at_ts(samples, ts, obs.field, obs.index)
        if val is None:
            return False  # 缺失数据视为不通过
        ctx[obs.name] = val
    result = filter_fn(ctx)
    return bool(result) if result is not None else False


def _build_context(ts, ir, state_dict, topic_data, param_resolver):
    """为单个采样点构建变量上下文"""
    ctx = {"_param_resolver": param_resolver, "_ts": ts,
           "_param_names": [p.name for p in ir.parameters]}

    # 绑定观测变量
    for obs in ir.observations:
        samples = topic_data.get(obs.topic, [])
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


def _get_field_value(msg, field_name, index=None):
    """从消息对象获取字段值"""
    val = getattr(msg, field_name, None)
    if val is None:
        return None
    if index is not None:
        try:
            val = val[index]
        except (IndexError, TypeError):
            return None
    return float(val) if val is not None else None


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


def _update_feedback(feedback_list, fb_spec, value):
    """更新 feedback 列表中对应的 feedback 实例"""
    for fb in feedback_list:
        if fb.name == fb_spec.name:
            fb.update_value(value)
            return


def load_compiled_oracles(
    target: str,
    spec_dir: Path | None = None,
    param_resolver: Callable[[str], float] | None = None,
):
    """
    加载并编译指定目标的所有 OracleIR spec。

    Args:
        target: 目标系统名（如 "px4", "turtlebot3"）
        spec_dir: spec 目录路径（默认 specs/{target}/）
        param_resolver: 外部参数解析器（可选）
    """
    from .parser import load_all_specs

    if spec_dir is None:
        spec_dir = Path(__file__).parent.parent / "specs" / target

    if not spec_dir.exists():
        return []

    specs = load_all_specs(spec_dir)
    return [compile_oracle_ir(ir, param_resolver) for ir in specs]


