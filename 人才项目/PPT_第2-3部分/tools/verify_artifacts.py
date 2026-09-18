from __future__ import annotations

import hashlib
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
SOURCE = REPO / "人才项目" / "拔尖人才项目阶段进展汇报.pptx"
OUTPUT = ROOT / "拔尖人才项目阶段进展汇报_RoboFuzz_第2-3部分.pptx"
PREVIEW = ROOT / "PPT_第2-3部分_预览.png"
EXPECTED_SOURCE_SHA256 = (
    "a70ebcbd6fced30465847595fa49fa566a8b340bc4fb2748d04e2537b843d91a"
)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def page_count(path):
    namespace = {
        "p": "http://schemas.openxmlformats.org/presentationml/2006/main"
    }
    with zipfile.ZipFile(path) as archive:
        root = ET.fromstring(archive.read("ppt/presentation.xml"))
    return len(root.findall(".//p:sldIdLst/p:sldId", namespace))


def ppt_text(path):
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


def render_preview():
    render_dir = Path("/tmp/robofuzz-progress-ppt-render")
    if render_dir.exists():
        shutil.rmtree(render_dir)
    render_dir.mkdir(parents=True)
    profile = Path(tempfile.mkdtemp(prefix="robofuzz-lo-render-"))
    try:
        subprocess.run(
            [
                "soffice",
                "--headless",
                "--nologo",
                "--norestore",
                f"-env:UserInstallation={profile.as_uri()}",
                "--convert-to",
                "pdf",
                "--outdir",
                str(render_dir),
                str(OUTPUT),
            ],
            check=True,
        )
    finally:
        shutil.rmtree(profile, ignore_errors=True)

    pdf = render_dir / f"{OUTPUT.stem}.pdf"
    subprocess.run(
        [
            "pdftoppm",
            "-png",
            "-r",
            "150",
            str(pdf),
            str(render_dir / "slide"),
        ],
        check=True,
    )
    pages = sorted(
        render_dir.glob("slide-*.png"),
        key=lambda path: int(re.search(r"\d+", path.stem).group()),
    )
    if len(pages) != 7:
        raise RuntimeError(f"expected 7 rendered pages, got {len(pages)}")
    subprocess.run(
        [
            "montage",
            *map(str, pages),
            "-background",
            "white",
            "-tile",
            "2x4",
            "-geometry",
            "960x540+12+12",
            str(PREVIEW),
        ],
        check=True,
    )
    return pdf


def main():
    source_hash = sha256(SOURCE)
    if source_hash != EXPECTED_SOURCE_SHA256:
        raise RuntimeError(f"source template hash changed: {source_hash}")
    if page_count(OUTPUT) != 7:
        raise RuntimeError("PPTX page count is not 7")

    text = ppt_text(OUTPUT)
    for forbidden in (
        "Peach",
        "语义流图",
        "协议代码测试并行执行",
        "留到绩效自评价报告",
    ):
        if forbidden in text:
            raise RuntimeError(f"forbidden template text remains: {forbidden}")
    for required in (
        "34 条 Issue 不等于 34 个漏洞",
        "组合优化结果，不等同于语义约束覆盖率",
        "并非151份IR均已逐条在线执行",
        "实物验证、未公开漏洞认定和正式编号尚未闭环",
    ):
        if required not in text:
            raise RuntimeError(f"required boundary missing: {required}")

    for stem in (
        "01_三任务总体链路",
        "02_规范到Oracle链路",
        "03_语义反馈与种子调度",
        "04_偏轨证据与根因归并",
    ):
        for suffix in (".svg", ".png"):
            asset = ROOT / "assets" / f"{stem}{suffix}"
            if not asset.is_file():
                raise RuntimeError(f"missing asset: {asset.name}")

    pdf = render_preview()
    info = subprocess.check_output(["pdfinfo", str(pdf)], text=True)
    if not re.search(r"^Pages:\s+7\s*$", info, re.MULTILINE):
        raise RuntimeError("rendered PDF is not 7 pages")

    print(f"source_sha256={source_hash}")
    print(f"output_sha256={sha256(OUTPUT)}")
    print(f"preview={PREVIEW}")


if __name__ == "__main__":
    main()
