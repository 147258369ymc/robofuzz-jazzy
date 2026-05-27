"""
预处理流水线编排 — 将探测、分块、归一化、索引构建串联为完整流程。
"""

from __future__ import annotations
import json
import logging
from pathlib import Path
from dataclasses import asdict

from .schema import SpecBlock
from .detector import DocTypeDetector, DocType
from .chunkers import (
    BaseChunker,
    StructuredDataChunker,
    TableMarkdownChunker,
    HeadingMarkdownChunker,
    ProtocolFlowChunker,
    YamlParamChunker,
    SourceCodeChunker,
    RosInterfaceChunker,
    RobotModelChunker,
)
from .normalizer import Normalizer
from .indexer import IndexBuilder, SpecIndex
from .config import TargetConfig, FileConfig

logger = logging.getLogger(__name__)


class PreprocessingPipeline:
    """
    多源规约预处理流水线。

    用法:
        from preprocessing import PreprocessingPipeline
        from preprocessing.config import PX4_CONFIG

        pipeline = PreprocessingPipeline(PX4_CONFIG)
        result = pipeline.run()
        result.save("system_doc/preprocessed/")
    """

    def __init__(self, config: TargetConfig, project_root: Path | None = None,
                 descriptor_path: Path | None = None):
        self.config = config
        self.project_root = project_root or Path.cwd()
        self.descriptor_path = descriptor_path
        self.detector = DocTypeDetector()
        self._chunkers: dict[str, BaseChunker] = {}
        self._setup_chunkers()

    def _setup_chunkers(self):
        """初始化各类型分块器"""
        self._chunkers = {
            DocType.STRUCTURED_DATA.value: StructuredDataChunker(
                json_array_key=self.config.json_array_key,
                xml_element_tag=self.config.xml_element_tag,
            ),
            DocType.YAML_PARAMS.value: YamlParamChunker(),
            DocType.TABULAR_MARKDOWN.value: TableMarkdownChunker(),
            DocType.PROSE_MARKDOWN.value: HeadingMarkdownChunker(),
            DocType.PROTOCOL_SPEC.value: ProtocolFlowChunker(),
            DocType.SOURCE_CODE.value: SourceCodeChunker(),
            DocType.ROS_INTERFACE.value: RosInterfaceChunker(),
            DocType.ROBOT_MODEL.value: RobotModelChunker(),
        }

    def register_chunker(self, doc_type: str, chunker: BaseChunker):
        """注册自定义分块器（用于扩展新文档类型）"""
        self._chunkers[doc_type] = chunker

    def run(self) -> PipelineResult:
        """执行完整预处理流水线"""
        doc_root = self.project_root / self.config.doc_root
        normalizer = Normalizer(
            source_system=self.config.name,
            version=self.config.version,
            base_path=str(self.project_root),
        )

        all_blocks: list[SpecBlock] = []
        file_stats: list[dict] = []

        # 确定要处理的文件
        files_to_process = self._resolve_files(doc_root)

        for file_path, file_config in files_to_process:
            logger.info(f"Processing: {file_path.name}")
            try:
                blocks = self._process_file(file_path, file_config, normalizer)
                all_blocks.extend(blocks)
                file_stats.append({
                    "file": str(file_path.relative_to(self.project_root)),
                    "blocks": len(blocks),
                    "status": "ok",
                })
            except Exception as e:
                logger.error(f"Failed to process {file_path}: {e}")
                file_stats.append({
                    "file": str(file_path.relative_to(self.project_root)),
                    "blocks": 0,
                    "status": f"error: {e}",
                })

        # 去重（同名同类型的 block 合并 ID）
        all_blocks = self._deduplicate_ids(all_blocks)

        # 构建索引（如果有 descriptor 则使用目标特定的标签规则）
        if self.descriptor_path and self.descriptor_path.exists():
            index_builder = IndexBuilder.from_descriptor(self.descriptor_path)
        else:
            index_builder = IndexBuilder()
        index = index_builder.build(all_blocks)

        return PipelineResult(
            blocks=all_blocks,
            index=index,
            config=self.config,
            stats=file_stats,
        )

    def _resolve_files(self, doc_root: Path) -> list[tuple[Path, FileConfig | None]]:
        """解析要处理的文件列表"""
        results = []
        if self.config.file_configs:
            for fc in self.config.file_configs:
                if fc.skip:
                    continue
                fp = doc_root / fc.path
                if fp.exists():
                    results.append((fp, fc))
                else:
                    logger.warning(f"File not found: {fp}")
        else:
            # 无显式配置时，递归发现所有可处理的文件
            skip_names = {"README.md", ".git", "__pycache__", ".gitignore"}
            skip_suffixes = {".stl", ".dae", ".png", ".jpg", ".pgm", ".pyc"}
            for fp in sorted(doc_root.rglob("*")):
                if not fp.is_file():
                    continue
                if fp.name in skip_names:
                    continue
                if fp.suffix.lower() in skip_suffixes:
                    continue
                if any(part.startswith(".") for part in fp.parts):
                    continue
                results.append((fp, None))
        return results

    def _process_file(
        self, file_path: Path, file_config: FileConfig | None, normalizer: Normalizer
    ) -> list[SpecBlock]:
        """处理单个文件：探测 → 分块 → 归一化"""
        content = file_path.read_text(encoding="utf-8")

        # 确定文档类型
        if file_config and file_config.chunker_override:
            doc_type = file_config.chunker_override
        else:
            detected = self.detector.detect(file_path, content)
            doc_type = detected.value

        # 获取分块器
        chunker = self._get_chunker(doc_type, file_config)

        # 分块
        raw_chunks = chunker.chunk(content, file_path)
        logger.info(f"  → {len(raw_chunks)} chunks ({doc_type})")

        # 归一化
        blocks = [normalizer.normalize(c, file_path, doc_type) for c in raw_chunks]
        return blocks

    def _get_chunker(self, doc_type: str, file_config: FileConfig | None) -> BaseChunker:
        """获取分块器实例，支持文件级参数覆盖"""
        if doc_type not in self._chunkers:
            raise ValueError(f"No chunker for doc_type '{doc_type}'")

        chunker = self._chunkers[doc_type]

        # 如果有文件级参数覆盖，创建新实例避免污染共享状态
        if file_config and file_config.chunker_params:
            import copy
            chunker = copy.copy(chunker)
            for key, val in file_config.chunker_params.items():
                if hasattr(chunker, key):
                    setattr(chunker, key, val)
        return chunker

    def _deduplicate_ids(self, blocks: list[SpecBlock]) -> list[SpecBlock]:
        """确保 block_id 唯一"""
        seen: dict[str, int] = {}
        for block in blocks:
            if block.block_id in seen:
                seen[block.block_id] += 1
                block.block_id = f"{block.block_id}_{seen[block.block_id]}"
            else:
                seen[block.block_id] = 0
        return blocks


class PipelineResult:
    """流水线执行结果"""

    def __init__(
        self,
        blocks: list[SpecBlock],
        index: SpecIndex,
        config: TargetConfig,
        stats: list[dict],
    ):
        self.blocks = blocks
        self.index = index
        self.config = config
        self.stats = stats

    @property
    def summary(self) -> str:
        type_counts = {}
        for b in self.blocks:
            type_counts[b.block_type] = type_counts.get(b.block_type, 0) + 1
        lines = [
            f"Target: {self.config.name} (v{self.config.version})",
            f"Total blocks: {len(self.blocks)}",
            f"Block types: {type_counts}",
            f"Index entities: {len(self.index.entity_index)}",
            f"Index tags: {len(self.index.tag_index)}",
        ]
        return "\n".join(lines)

    def save(self, output_dir: str | Path):
        """保存预处理结果到磁盘"""
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        blocks_dir = out / "blocks"
        blocks_dir.mkdir(exist_ok=True)

        # 保存每个 block
        for block in self.blocks:
            fname = f"{block.block_id}.json"
            fname = fname.replace("/", "_").replace("\\", "_")
            (blocks_dir / fname).write_text(block.to_json(), encoding="utf-8")

        # 保存索引
        self.index.save(out / "index.json")

        # 保存 manifest
        manifest = {
            "target": self.config.name,
            "version": self.config.version,
            "total_blocks": len(self.blocks),
            "file_stats": self.stats,
        }
        (out / "manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
        )

    def get_blocks_by_type(self, block_type: str) -> list[SpecBlock]:
        return [b for b in self.blocks if b.block_type == block_type]

    def get_blocks_by_tag(self, tag: str) -> list[SpecBlock]:
        ids = set(self.index.query_tag(tag))
        return [b for b in self.blocks if b.block_id in ids]

    def find_entity(self, name: str) -> list[SpecBlock]:
        ids = set(self.index.query_entity(name))
        return [b for b in self.blocks if b.block_id in ids]
