"""受限表达式引擎 — 解析、校验、编译 OracleIR 表达式为可执行 Python"""

from __future__ import annotations
import ast
import math
import operator
from typing import Any, Callable


# 允许的内置函数
BUILTIN_FUNCTIONS = {
    "sqrt": math.sqrt,
    "abs": abs,
    "min": min,
    "max": max,
    "norm": lambda *args: math.sqrt(sum(x * x for x in args)),
    "mean": lambda *args: sum(args) / len(args) if args else 0.0,
    "degrees": math.degrees,
    "radians": math.radians,
    "acos": math.acos,
    "isnan": math.isnan,
    "isinf": math.isinf,
    "is_valid": lambda x: not (math.isnan(x) or math.isinf(x)),
}

# 允许的运算符
ALLOWED_OPS = {
    ast.Add: operator.add,
    ast.Sub: operator.sub,
    ast.Mult: operator.mul,
    ast.Div: operator.truediv,
    ast.Mod: operator.mod,
    ast.Pow: operator.pow,
    ast.USub: operator.neg,
}

ALLOWED_COMPARATORS = {
    ast.Lt: operator.lt,
    ast.LtE: operator.le,
    ast.Gt: operator.gt,
    ast.GtE: operator.ge,
    ast.Eq: operator.eq,
    ast.NotEq: operator.ne,
}


class ExprError(Exception):
    """表达式解析/编译错误"""
    pass


class ExprValidator:
    """验证表达式是否在受限语言范围内"""

    ALLOWED_NODE_TYPES = (
        ast.Expression, ast.BinOp, ast.UnaryOp, ast.Compare,
        ast.Call, ast.Name, ast.Constant, ast.Attribute,
        ast.BoolOp, ast.Subscript, ast.Index, ast.Tuple,
    )

    def validate(self, expr_str: str) -> list[str]:
        """返回错误列表，空列表表示合法"""
        errors = []
        try:
            tree = ast.parse(expr_str, mode="eval")
        except SyntaxError as e:
            return [f"Syntax error: {e}"]

        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                if isinstance(node.func, ast.Name):
                    if node.func.id not in BUILTIN_FUNCTIONS and node.func.id != "param":
                        errors.append(f"Unknown function: {node.func.id}")
                elif not isinstance(node.func, ast.Attribute):
                    errors.append(f"Invalid call: {ast.dump(node.func)}")
            elif isinstance(node, (ast.Import, ast.ImportFrom)):
                errors.append("Import not allowed")
            elif isinstance(node, (ast.Lambda, ast.FunctionDef)):
                errors.append("Function definition not allowed")
            elif isinstance(node, (ast.For, ast.While, ast.If)):
                errors.append("Control flow not allowed")
        return errors


class ExprCompiler:
    """将受限表达式编译为可执行的 Python callable"""

    def compile(self, expr_str: str, var_names: list[str]) -> Callable:
        """
        编译表达式为函数。

        Args:
            expr_str: 表达式字符串
            var_names: 可用变量名列表

        Returns:
            callable(context: dict) -> Any
        """
        validator = ExprValidator()
        errors = validator.validate(expr_str)
        if errors:
            raise ExprError(f"Expression '{expr_str}' invalid: {errors}")

        # 构建安全的执行环境
        safe_globals = {"__builtins__": {}}
        safe_globals.update(BUILTIN_FUNCTIONS)
        safe_globals["param"] = lambda name: None  # placeholder

        # 编译为 code object
        try:
            code = compile(expr_str, "<oracle_ir_expr>", "eval")
        except SyntaxError as e:
            raise ExprError(f"Compile error: {e}")

        def evaluator(context: dict) -> Any:
            """执行表达式，context 包含所有变量绑定"""
            local_env = dict(context)
            local_env.update(BUILTIN_FUNCTIONS)
            # param() 解析器：支持 param(NAME) 其中 NAME 是标识符
            # 当 NAME 已在 env 中绑定为数值时，说明它是参数本身的值
            resolver = context.get("_param_resolver", lambda n: 0.0)
            param_names = set(context.get("_param_names", []))

            def param_fn(name_or_value):
                """如果传入的是参数名(str)则查找，如果是数值则说明变量已解析为值，直接返回"""
                if isinstance(name_or_value, str):
                    return resolver(name_or_value)
                # name_or_value 是数值 — 说明 param(MPC_XY_VEL_MAX) 中
                # MPC_XY_VEL_MAX 已被解析为其默认值，直接返回
                return name_or_value

            local_env["param"] = param_fn
            # 将参数名放入环境（值为其默认值）
            if "_param_names" in context:
                for pname in context["_param_names"]:
                    if pname not in local_env:
                        local_env[pname] = resolver(pname)
            try:
                return eval(code, safe_globals, local_env)
            except (ZeroDivisionError, ValueError, TypeError, NameError):
                return None

        return evaluator


def compile_expr(expr_str: str, var_names: list[str] | None = None) -> Callable:
    """便捷函数：编译表达式"""
    return ExprCompiler().compile(expr_str, var_names or [])


def validate_expr(expr_str: str) -> list[str]:
    """便捷函数：验证表达式"""
    return ExprValidator().validate(expr_str)

