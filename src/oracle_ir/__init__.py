"""
OracleIR — 可追溯 Oracle 中间表示

Agent 与 Fuzzer 之间的形式化中间层。
Agent 输出 OracleIR YAML，系统校验后编译为可执行 oracle。
"""

from .schema import (
    OracleIR, Observation, Parameter, DerivedVar,
    Assertion, Scope, Window, FeedbackSpec, ProvenanceRef,
)
from .parser import load_oracle_ir, load_all_specs
from .validator import validate_oracle_ir
from .compiler import compile_oracle_ir

__all__ = [
    "OracleIR", "Observation", "Parameter", "DerivedVar",
    "Assertion", "Scope", "Window", "FeedbackSpec", "ProvenanceRef",
    "load_oracle_ir", "load_all_specs",
    "validate_oracle_ir", "compile_oracle_ir",
]
