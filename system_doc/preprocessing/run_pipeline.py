#!/usr/bin/env python3
"""
运行预处理流水线的入口脚本。

用法:
    python -m system_doc.preprocessing.run_pipeline --target px4
    python -m system_doc.preprocessing.run_pipeline --target px4 --output system_doc/preprocessed
"""

import argparse
import logging
import sys
from pathlib import Path

# 将项目根目录加入 path
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from system_doc.preprocessing.pipeline import PreprocessingPipeline
from system_doc.preprocessing.config import get_config


def main():
    parser = argparse.ArgumentParser(description="多源规约预处理流水线")
    parser.add_argument(
        "--target", required=True,
        help="目标系统名称 (px4, turtlebot3)",
    )
    parser.add_argument(
        "--output", default=None,
        help="输出目录 (默认: system_doc/preprocessed/<target>)",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="显示详细日志",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s: %(message)s",
    )

    config = get_config(args.target)
    output_dir = args.output or f"system_doc/preprocessed/{args.target}"

    pipeline = PreprocessingPipeline(config, project_root=PROJECT_ROOT)
    result = pipeline.run()

    print(result.summary)
    result.save(PROJECT_ROOT / output_dir)
    print(f"\nSaved to: {output_dir}/")


if __name__ == "__main__":
    main()
