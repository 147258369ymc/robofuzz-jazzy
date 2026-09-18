from __future__ import annotations

import hashlib
import shutil
from pathlib import Path

import content
from uno_ppt import (
    ImpressSession,
    add_svg,
    add_text,
    clear_page,
    cm,
    color_prefix,
    make_table,
    set_cell,
)


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
SOURCE = REPO / "人才项目" / "拔尖人才项目阶段进展汇报.pptx"
OUTPUT = ROOT / "拔尖人才项目阶段进展汇报_RoboFuzz_第2-3部分.pptx"
ASSETS = ROOT / "assets"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def walk_shapes(container):
    for index in range(container.getCount()):
        shape = container.getByIndex(index)
        yield shape
        if hasattr(shape, "getCount"):
            yield from walk_shapes(shape)


def replace_text(page, replacements):
    for shape in walk_shapes(page):
        if not hasattr(shape, "String"):
            continue
        text = shape.String
        for old, new in replacements.items():
            text = text.replace(old, new)
        shape.String = text


def build_metrics_slide(document, page):
    clear_page(page)
    add_text(
        document,
        page,
        1.86,
        1.29,
        29.50,
        1.80,
        "任务与指标完成情况",
        32,
        bold=True,
    )
    add_text(
        document,
        page,
        0.50,
        3.32,
        32.87,
        1.00,
        "课题：具身智能系统控制逻辑漏洞挖掘技术研究",
        18,
        bold=True,
        align=3,
        valign=1,
        border=0x000000,
        fill=0x00B0F0,
    )

    _, table = make_table(document, page, 0.50, 4.32, 32.87, 13.35, 4, 7)
    column_widths_cm = [4.27, 6.57, 7.23, 14.80]
    row_heights_cm = [1.20, 2.30, 2.18, 1.62, 2.05, 2.00, 2.00]
    for index, width in enumerate(column_widths_cm):
        table.getColumns().getByIndex(index).Width = cm(width)
    for index, height in enumerate(row_heights_cm):
        table.getRows().getByIndex(index).Height = cm(height)

    headers = ["指标名称", "原始基线 / 立项状态", "开题完成要求", "当前指标状态"]
    for column, value in enumerate(headers):
        set_cell(
            table.getCellByPosition(column, 0),
            value,
            font_size=14,
            color=0xC00000 if column == 3 else 0x000000,
            bold=True,
            fill=0xFFFFFF,
            align=3,
            valign=1,
        )

    for row_index, metric in enumerate(content.METRICS, start=1):
        values = (
            metric["name"],
            metric["baseline"],
            metric["requirement"],
            metric["current"],
        )
        for column, value in enumerate(values):
            set_cell(
                table.getCellByPosition(column, row_index),
                value,
                font_size=11.5,
                color=0xC00000 if column == 3 else 0x000000,
                bold=column == 0,
                fill=0xFFFFFF,
                align=3 if column == 0 else 0,
                valign=1,
            )

    add_text(
        document,
        page,
        0.75,
        17.78,
        32.20,
        0.72,
        content.FOOTNOTE,
        11,
        color=0x5F6B76,
        align=3,
        valign=1,
    )


def build_technical_slide(session, document, page, spec):
    clear_page(page)
    title_size = 30 if len(spec["title"]) > 28 else 32
    add_text(
        document,
        page,
        1.86,
        1.29,
        29.80,
        1.80,
        spec["title"],
        title_size,
        bold=True,
    )
    add_text(
        document,
        page,
        1.88,
        3.65,
        4.23,
        1.66,
        "主要难点",
        24,
        color=0x002060,
        bold=True,
        align=3,
        valign=1,
        border=0x2D2D8A,
        fill=0xFFFFFF,
    )
    difficulty = add_text(
        document,
        page,
        6.10,
        3.65,
        25.43,
        1.66,
        f"□ {spec['difficulty']}",
        23,
        bold=True,
        align=0,
        valign=1,
        border=0x002060,
        fill=0xFFFFFF,
        dash=True,
    )
    prefix = (
        f"□ {spec['difficulty'].split('：', 1)[0]}："
        if "：" in spec["difficulty"]
        else "□ "
    )
    color_prefix(difficulty, prefix)

    add_svg(
        session,
        document,
        page,
        ASSETS / spec["asset"],
        1.60,
        5.42,
        30.50,
        10.17,
    )
    add_text(
        document,
        page,
        1.88,
        15.94,
        3.51,
        2.65,
        "方 法\n创 新",
        24,
        color=0x002060,
        bold=True,
        align=3,
        valign=1,
        border=0x2D2D8A,
        fill=0xFFFFFF,
    )
    method_size = 18 if max(map(len, spec["method_lines"])) > 62 else 20
    add_text(
        document,
        page,
        5.58,
        15.94,
        25.95,
        1.18,
        f"□ {spec['method_lines'][0]}",
        method_size,
        bold=True,
        align=0,
        valign=1,
        border=0x002060,
        fill=0xFFFFFF,
        dash=True,
    )
    add_text(
        document,
        page,
        5.58,
        17.15,
        25.95,
        1.08,
        f"□ {spec['method_lines'][1]}",
        method_size,
        bold=True,
        align=0,
        valign=1,
        border=0x002060,
        fill=0xFFFFFF,
        dash=True,
    )
    add_text(
        document,
        page,
        5.58,
        18.26,
        25.95,
        0.48,
        spec["boundary"],
        12.5,
        color=0x5F6B76,
        align=0,
        valign=1,
    )


def main():
    ROOT.mkdir(parents=True, exist_ok=True)
    source_hash = sha256(SOURCE)
    print(f"source_sha256_before={source_hash}")

    lock_path = OUTPUT.with_name(f".~lock.{OUTPUT.name}#")
    if lock_path.exists():
        lock_owner = lock_path.read_text(encoding="utf-8", errors="replace")
        if "robofuzz-lo-profile-" not in lock_owner:
            raise RuntimeError(
                f"output appears to be open in another LibreOffice instance: {lock_path}"
            )
        lock_path.unlink()
    shutil.copy2(SOURCE, OUTPUT)

    with ImpressSession() as session:
        document = session.open(OUTPUT)
        try:
            pages = document.getDrawPages()
            if pages.getCount() != 8:
                raise RuntimeError(
                    f"expected 8 template slides, got {pages.getCount()}"
                )

            pages.remove(pages.getByIndex(0))
            replacements = {
                "课题概要与研究内容": "课题概况与研究内容",
                "年度进展与重要成果": "技术细节",
            }
            replace_text(pages.getByIndex(0), replacements)
            build_metrics_slide(document, pages.getByIndex(1))
            replace_text(pages.getByIndex(2), replacements)
            for offset, spec in enumerate(content.TECH_SLIDES, start=3):
                build_technical_slide(
                    session,
                    document,
                    pages.getByIndex(offset),
                    spec,
                )
            session.save_and_close(document)
            document = None
        finally:
            if document is not None:
                session.close(document)

    if sha256(SOURCE) != source_hash:
        raise RuntimeError("source template changed during build")
    print(f"output={OUTPUT}")


if __name__ == "__main__":
    main()
