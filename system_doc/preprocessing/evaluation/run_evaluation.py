#!/usr/bin/env python3
"""
运行预处理评估的入口脚本。

用法:
    python -m system_doc.preprocessing.evaluation.run_evaluation --target px4
    python -m system_doc.preprocessing.evaluation.run_evaluation --target px4 --verbose
"""

import argparse
import logging
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from system_doc.preprocessing.pipeline import PreprocessingPipeline
from system_doc.preprocessing.config import get_config
from system_doc.preprocessing.evaluation.runner import EvaluationRunner


def main():
    parser = argparse.ArgumentParser(description="预处理流水线评估")
    parser.add_argument("--target", required=True, help="目标系统名称")
    parser.add_argument("--verbose", "-v", action="store_true")
    parser.add_argument("--rerun", action="store_true", help="重新运行流水线（而非使用缓存）")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s: %(message)s",
    )

    config = get_config(args.target)

    # 运行流水线
    logging.info("Running preprocessing pipeline...")
    pipeline = PreprocessingPipeline(config, project_root=PROJECT_ROOT)
    result = pipeline.run()
    logging.info(f"Pipeline produced {len(result.blocks)} blocks")

    # 运行评估
    logging.info("Running evaluation...")
    runner = EvaluationRunner(result, config, project_root=PROJECT_ROOT)
    report = runner.run_all()

    # 输出报告
    print(report.format_report())

    # 返回码
    sys.exit(0 if report.all_passed else 1)


if __name__ == "__main__":
    main()
