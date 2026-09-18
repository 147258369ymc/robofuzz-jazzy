import hashlib
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
TOOLS = ROOT / "tools"
SOURCE = REPO / "人才项目" / "拔尖人才项目阶段进展汇报.pptx"
OUTPUT = ROOT / "拔尖人才项目阶段进展汇报_RoboFuzz_第2-3部分.pptx"

sys.path.insert(0, str(TOOLS))

import uno_ppt


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def ppt_text(path: Path) -> str:
    paragraphs = []
    with zipfile.ZipFile(path) as archive:
        slide_names = sorted(
            (
                name
                for name in archive.namelist()
                if re.fullmatch(r"ppt/slides/slide\d+\.xml", name)
            ),
            key=lambda name: int(re.search(r"\d+", name).group()),
        )
        for name in slide_names:
            root = ET.fromstring(archive.read(name))
            for paragraph in root.iter():
                if not paragraph.tag.endswith("}p"):
                    continue
                paragraphs.append(
                    "".join(
                        node.text or ""
                        for node in paragraph.iter()
                        if node.tag.endswith("}t")
                    )
                )
    return "\n".join(paragraphs)


def presentation_page_count(path: Path) -> int:
    namespace = {
        "p": "http://schemas.openxmlformats.org/presentationml/2006/main"
    }
    with zipfile.ZipFile(path) as archive:
        root = ET.fromstring(archive.read("ppt/presentation.xml"))
    return len(root.findall(".//p:sldIdLst/p:sldId", namespace))


class UnoPptTests(unittest.TestCase):
    def test_enter_removes_profile_if_socket_creation_fails(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            profile = Path(temporary_directory) / "profile"
            profile.mkdir()
            session = uno_ppt.ImpressSession()
            with (
                patch.object(uno_ppt.tempfile, "mkdtemp", return_value=str(profile)),
                patch.object(
                    uno_ppt.socket, "socket", side_effect=PermissionError("blocked")
                ),
                self.assertRaises(PermissionError),
            ):
                session.__enter__()
            self.assertFalse(profile.exists())

    def test_editing_copy_does_not_change_source(self):
        source_hash = sha256(SOURCE)
        with tempfile.TemporaryDirectory() as temporary_directory:
            copy_path = Path(temporary_directory) / "copy.pptx"
            shutil.copy2(SOURCE, copy_path)

            with uno_ppt.ImpressSession() as session:
                document = session.open(copy_path)
                self.assertEqual(document.getDrawPages().getCount(), 8)
                document.getDrawPages().remove(document.getDrawPages().getByIndex(0))
                session.save_and_close(document)

            with uno_ppt.ImpressSession() as session:
                document = session.open(copy_path)
                self.assertEqual(document.getDrawPages().getCount(), 7)
                session.close(document)

        self.assertEqual(sha256(SOURCE), source_hash)


class FinalBuildTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        subprocess.run(
            [sys.executable, str(TOOLS / "generate_diagrams.py")], check=True
        )
        environment = dict(os.environ)
        environment["PYTHONPATH"] = "/usr/lib/python3/dist-packages"
        subprocess.run(
            [sys.executable, str(TOOLS / "build_ppt.py")],
            check=True,
            env=environment,
        )

    def test_final_presentation_has_seven_pages(self):
        self.assertEqual(presentation_page_count(OUTPUT), 7)

    def test_final_titles_and_boundaries_exist(self):
        text = ppt_text(OUTPUT)
        for phrase in (
            "任务与指标完成情况",
            "三项重点技术任务与总体链路",
            "规范驱动的语义约束提取与行为 Oracle 构建",
            "基于语义反馈的扰动输入生成与种子调度",
            "控制语义偏轨判定、证据回溯与根因归并",
            "34 条 Issue 不等于 34 个漏洞",
        ):
            self.assertIn(phrase, text)

    def test_template_example_text_is_removed(self):
        text = ppt_text(OUTPUT)
        for phrase in (
            "Peach",
            "语义流图",
            "协议代码测试并行执行",
            "留到绩效自评价报告",
        ):
            self.assertNotIn(phrase, text)


if __name__ == "__main__":
    unittest.main()
