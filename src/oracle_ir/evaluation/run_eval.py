"""OracleIR 评估运行器 — 串联 4 个维度，输出完整报告"""

from __future__ import annotations
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from src.oracle_ir.transform.parser import load_all_specs
from src.oracle_ir.evaluation import EvalReport
from src.oracle_ir.evaluation.d1_provenance import evaluate_provenance
from src.oracle_ir.evaluation.d2_semantics import evaluate_semantics
from src.oracle_ir.evaluation.d4_logic import evaluate_logic


def run_evaluation(
    spec_dir: Path | None = None,
    index_path: Path | None = None,
    blocks_dir: Path | None = None,
) -> EvalReport:
    """运行完整的 4 维评估"""

    # 默认路径
    project_root = Path(__file__).parent.parent.parent.parent
    if spec_dir is None:
        spec_dir = project_root / "src" / "oracle_ir" / "specs" / "px4"
    if index_path is None:
        index_path = project_root / "system_doc" / "preprocessed" / "px4" / "index.json"
    if blocks_dir is None:
        blocks_dir = project_root / "system_doc" / "preprocessed" / "px4" / "blocks"

    # 加载数据
    specs = load_all_specs(spec_dir)
    index_data = json.loads(index_path.read_text())

    print(f"Loaded {len(specs)} OracleIR specs")
    print(f"Loaded SpecIndex with {len(index_data.get('entity_index', {}))} entities")
    print()

    # 运行 3 个维度
    report = EvalReport()

    d1 = evaluate_provenance(specs, index_data)
    report.dimensions.append(d1)

    d2 = evaluate_semantics(specs, index_data, blocks_dir)
    report.dimensions.append(d2)

    d3 = evaluate_logic(specs, index_data, blocks_dir)
    report.dimensions.append(d3)

    report.compute_overall()
    return report


def print_report(report: EvalReport):
    """格式化输出评估报告"""
    print("=" * 70)
    print("  OracleIR Evaluation Report")
    print("  Ground Truth: SpecIndex (preprocessed documentation)")
    print("=" * 70)
    print()

    for dim in report.dimensions:
        pct = dim.score * 100
        bar = "#" * int(pct / 5) + "-" * (20 - int(pct / 5))
        print(f"  {dim.name}")
        print(f"    Score: {dim.passed}/{dim.total} = {pct:.1f}%  [{bar}]")
        if dim.failures:
            print(f"    Failures ({len(dim.failures)}):")
            for f in dim.failures[:5]:
                print(f"      {f}")
            if len(dim.failures) > 5:
                print(f"      ... and {len(dim.failures) - 5} more")
        print()

    print("-" * 70)
    print(f"  Overall Score: {report.overall_score * 100:.1f}%")
    print("-" * 70)
    print()

    # 解读
    print("  Interpretation:")
    print("    D1 (Provenance): Can we trace each oracle back to source docs?")
    print("    D2 (Semantics):  Are parameter values/units factually correct?")
    print("    D3 (Logic):      Is the assertion direction consistent with spec semantics?")


if __name__ == "__main__":
    report = run_evaluation()
    print_report(report)
