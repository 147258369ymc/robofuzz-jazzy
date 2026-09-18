from __future__ import annotations

import subprocess
from pathlib import Path

from svg_diagrams import (
    OUTPUT_STEMS,
    draw_evidence_root_cause,
    draw_oracle_pipeline,
    draw_overview,
    draw_seed_feedback,
)


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets"
BUILDERS = {
    "01_三任务总体链路": draw_overview,
    "02_规范到Oracle链路": draw_oracle_pipeline,
    "03_语义反馈与种子调度": draw_seed_feedback,
    "04_偏轨证据与根因归并": draw_evidence_root_cause,
}


def main() -> None:
    missing = set(OUTPUT_STEMS) - set(BUILDERS)
    extra = set(BUILDERS) - set(OUTPUT_STEMS)
    if missing or extra:
        raise ValueError(f"BUILDERS/OUTPUT_STEMS mismatch: missing={sorted(missing)}, extra={sorted(extra)}")
    ASSETS.mkdir(parents=True, exist_ok=True)
    for stem in OUTPUT_STEMS:
        svg_path = ASSETS / f"{stem}.svg"
        png_path = ASSETS / f"{stem}.png"
        svg_path.write_text(BUILDERS[stem](), encoding="utf-8")
        subprocess.run(
            ["rsvg-convert", "-w", "2400", "-h", "800", "-o", str(png_path), str(svg_path)],
            check=True,
        )


if __name__ == "__main__":
    main()
