"""预处理流水线 CLI 入口"""
import sys
import argparse
import logging

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[2]))

from system_doc.preprocessing.pipeline import PreprocessingPipeline
from system_doc.preprocessing.config import PX4_CONFIG, TURTLEBOT3_CONFIG

TARGETS = {
    "px4": PX4_CONFIG,
    "turtlebot3": TURTLEBOT3_CONFIG,
}


def main():
    parser = argparse.ArgumentParser(description="规范文档预处理")
    parser.add_argument("target", choices=TARGETS.keys(), help="目标平台")
    parser.add_argument("-o", "--output", help="输出目录（默认 system_doc/preprocessed/<target>/）")
    parser.add_argument("-v", "--verbose", action="store_true", help="显示详细日志")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING)

    config = TARGETS[args.target]
    pipeline = PreprocessingPipeline(config)
    result = pipeline.run()

    output_dir = args.output or f"system_doc/preprocessed/{args.target}/"
    result.save(output_dir)

    print(result.summary)
    print(f"Saved to: {output_dir}")


if __name__ == "__main__":
    main()
