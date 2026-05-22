"""评估编排器 — 串联所有评估维度"""

from __future__ import annotations
import logging
from pathlib import Path

from ..schema import SpecBlock
from ..indexer import SpecIndex
from ..pipeline import PipelineResult
from ..config import TargetConfig
from .report import EvaluationReport, DimensionResult
from .ground_truth import (
    GroundTruthEntry, GroundTruthConfig,
    get_ground_truth_configs, get_extractor,
)
from .metrics import (
    evaluate_chunking,
    evaluate_normalization,
    evaluate_references,
    evaluate_tags,
    evaluate_index,
)

logger = logging.getLogger(__name__)


class EvaluationRunner:
    """
    评估编排器。

    用法:
        from preprocessing.evaluation import EvaluationRunner
        runner = EvaluationRunner(pipeline_result, config)
        report = runner.run_all()
        print(report.format_report())
    """

    def __init__(
        self,
        pipeline_result: PipelineResult,
        config: TargetConfig,
        project_root: Path | None = None,
        gt_configs: list[GroundTruthConfig] | None = None,
    ):
        self.result = pipeline_result
        self.config = config
        self.project_root = project_root or Path.cwd()
        self.gt_configs = gt_configs or get_ground_truth_configs(config.name)
        self._ground_truth: dict[str, list[GroundTruthEntry]] | None = None

    def _load_ground_truth(self) -> dict[str, list[GroundTruthEntry]]:
        """加载所有 ground truth"""
        if self._ground_truth is not None:
            return self._ground_truth

        doc_root = self.project_root / self.config.doc_root
        gt: dict[str, list[GroundTruthEntry]] = {}

        for gt_config in self.gt_configs:
            file_path = doc_root / gt_config.file_path
            if not file_path.exists():
                logger.warning(f"Ground truth file not found: {file_path}")
                continue
            try:
                extractor = get_extractor(gt_config.extractor)
                entries = extractor(file_path, gt_config.params)
                gt[str(file_path)] = entries
                logger.info(f"  GT: {gt_config.file_path} → {len(entries)} entities")
            except Exception as e:
                logger.error(f"  GT extraction failed for {gt_config.file_path}: {e}")

        self._ground_truth = gt
        return gt

    def _all_gt_entries(self) -> list[GroundTruthEntry]:
        gt = self._load_ground_truth()
        return [e for entries in gt.values() for e in entries]

    def _known_entity_names(self) -> set[str]:
        """所有已知实体名的并集"""
        names = set()
        for entries in self._load_ground_truth().values():
            for e in entries:
                names.add(e.name)
        # 也加入 pipeline 输出中的所有 block name
        for block in self.result.blocks:
            names.add(block.name)
        return names

    def run_all(self) -> EvaluationReport:
        """运行所有评估维度"""
        logger.info("Loading ground truth...")
        gt = self._load_ground_truth()

        report = EvaluationReport(
            target=self.config.name,
            version=self.config.version,
            total_blocks=len(self.result.blocks),
        )

        # 维度1: 分块完整性
        logger.info("Evaluating chunking recall...")
        block_names = {b.name for b in self.result.blocks}
        report.dimensions.append(evaluate_chunking(gt, block_names))

        # 维度2: 归一化准确性
        logger.info("Evaluating normalization accuracy...")
        report.dimensions.append(evaluate_normalization(
            self.result.blocks, self._all_gt_entries()
        ))

        # 维度3: 引用精度
        logger.info("Evaluating reference precision...")
        report.dimensions.append(evaluate_references(
            self.result.blocks, self._known_entity_names(), target=self.config.name
        ))

        # 维度4: 标签质量
        logger.info("Evaluating tag quality...")
        report.dimensions.append(evaluate_tags(self.result.blocks))

        # 维度5: 索引正确性
        logger.info("Evaluating index correctness...")
        report.dimensions.append(evaluate_index(self.result.index, gt))

        return report
