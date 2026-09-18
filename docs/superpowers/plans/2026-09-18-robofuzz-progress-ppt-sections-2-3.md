# RoboFuzz 阶段汇报 PPT 第 2、3 部分 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 基于既有阶段汇报模板生成一份七页、可直接插入正式汇报的 PPTX，完整呈现六项指标和三项技术任务，并生成四张高信息密度技术图、PNG 预览及内容说明。

**Architecture:** 内容、图形和演示文稿编辑三层分离：`content.py` 固化所有文字和成果口径，`svg_diagrams.py` 生成四张可独立复用的 SVG/PNG，`build_ppt.py` 通过 LibreOffice UNO 编辑模板副本。验证层同时检查源模板哈希、PPT 页数与文本、SVG 几何规则、PDF 可渲染性和七页总览图，确保不会修改原模板或夸大成果。

**Tech Stack:** Python 3 标准库、LibreOffice 7.3 UNO、SVG、`rsvg-convert`、`soffice --headless`、Poppler `pdftoppm/pdfinfo`、ImageMagick `montage`、`unittest`。

---

## 1. 文件结构与职责

实施后只在以下目录新增或修改文件：

```text
人才项目/PPT_第2-3部分/
├── assets/
│   ├── 01_三任务总体链路.svg
│   ├── 01_三任务总体链路.png
│   ├── 02_规范到Oracle链路.svg
│   ├── 02_规范到Oracle链路.png
│   ├── 03_语义反馈与种子调度.svg
│   ├── 03_语义反馈与种子调度.png
│   ├── 04_偏轨证据与根因归并.svg
│   └── 04_偏轨证据与根因归并.png
├── tests/
│   ├── test_content.py
│   ├── test_diagrams.py
│   └── test_ppt_build.py
├── tools/
│   ├── content.py
│   ├── svg_diagrams.py
│   ├── generate_diagrams.py
│   ├── uno_ppt.py
│   ├── build_ppt.py
│   └── verify_artifacts.py
├── PPT_CONTENT_NOTES.md
├── PPT_第2-3部分_预览.png
└── 拔尖人才项目阶段进展汇报_RoboFuzz_第2-3部分.pptx
```

源模板保持不变：

```text
人才项目/拔尖人才项目阶段进展汇报.pptx
```

设计依据：

```text
docs/superpowers/specs/2026-09-18-robofuzz-progress-ppt-sections-2-3-design.md
```

构建过程中的 PDF 和逐页 PNG 只放入 `/tmp/robofuzz-progress-ppt-render/`，最终目录不保留临时文件，也不生成压缩包。

## 2. 全局安全与实现约束

- 绝不原地打开或保存源模板；先用 `shutil.copy2` 创建输出副本，再用独立 LibreOffice 用户配置编辑副本。
- 构建前后分别计算源模板 SHA-256，必须一致。
- 当前工作区已有改动均视为用户资产；每次提交只暂存本任务列出的文件。
- SVG 画布统一为 `2400 × 800`，插入 PPT 后约为 `30.5 × 10.2 cm`。
- SVG 一级标题字号不低于 `56` 个 viewBox 单位，内部字段不低于 `42` 个单位；换算到 PPT 后分别约为 20 pt 和 15 pt。
- 所有 `<rect>` 禁止设置 `rx`、`ry`；所有流程箭头必须由水平或垂直线段组成。
- 页面标题、难点带和方法创新带使用 PPT 原生可编辑形状；中央技术图以 SVG 插入，并保留同版 PNG。
- 指标表格使用 PPT 原生 `TableShape`，不能截图或栅格化。
- 任何自动检查不通过时，不得声称成品完成。

---

### Task 1: 固化内容清单并建立口径测试

**Files:**
- Create: `人才项目/PPT_第2-3部分/tools/content.py`
- Create: `人才项目/PPT_第2-3部分/tests/test_content.py`

- [ ] **Step 1: 创建内容测试，先固定页数、指标状态和禁止表述**

在 `test_content.py` 写入：

```python
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import content


class ContentTests(unittest.TestCase):
    def test_seven_pages_and_six_metrics(self):
        self.assertEqual(len(content.SLIDES), 7)
        self.assertEqual(len(content.METRICS), 6)

    def test_metric_states_match_design(self):
        states = [row["status"] for row in content.METRICS]
        self.assertEqual(
            states,
            ["达到数量规模", "部分达到", "达到：8类", "仿真侧达到", "待形成正式证据", "待形成正式证据"],
        )

    def test_three_tasks_only(self):
        self.assertEqual([item["id"] for item in content.TASKS], [1, 2, 3])

    def test_natural_method_copy_is_preserved(self):
        joined = "\n".join(
            line for slide in content.TECH_SLIDES for line in slide["method_lines"]
        )
        self.assertIn("先按文件类型分别处理", joined)
        self.assertIn("测试输入不再局限于修改一条消息的数值", joined)
        self.assertIn("发现偏轨后，同时保存测试输入", joined)
        self.assertNotIn("检索—生成—校验", joined)
        self.assertNotIn("错误签名—共现—级联去重", joined)

    def test_forbidden_claims_are_absent(self):
        corpus = content.all_visible_text()
        forbidden = (
            "34个漏洞",
            "34 个漏洞已发现",
            "151条Oracle全部执行",
            "151 条 Oracle 全部执行",
            "覆盖率提升33.9%",
            "覆盖率提升 33.9%",
            "已完成虚实差分",
            "四类输入全部完成",
        )
        for phrase in forbidden:
            self.assertNotIn(phrase, corpus)

    def test_required_boundaries_are_present(self):
        corpus = content.all_visible_text()
        for phrase in (
            "34 条 Issue 不等于 34 个漏洞",
            "组合优化结果，不等同于语义约束覆盖率",
            "成熟度不一",
            "下一阶段｜尚未开展",
        ):
            self.assertIn(phrase, corpus)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行测试并确认因 `content` 不存在而失败**

Run:

```bash
python3 -m unittest 人才项目/PPT_第2-3部分/tests/test_content.py -v
```

Expected: `ModuleNotFoundError: No module named 'content'`。

- [ ] **Step 3: 创建集中式内容清单**

在 `content.py` 中实现以下常量和辅助函数。指标正文必须直接采用已经确认的设计口径，不在构建脚本中二次改写：

```python
from __future__ import annotations

TOC = [
    "课题概况与研究内容",
    "任务与指标完成情况",
    "技术细节",
    "课题管理与风险分析",
]

METRICS = [
    {
        "name": "控制语义约束\n不少于50条",
        "baseline": "少量目标专用手写 Oracle；缺少统一的多源规范处理和结构化中间表示。",
        "requirement": "提取的控制语义约束不少于50条。",
        "status": "达到数量规模",
        "current": "达到数量规模。处理425个输入文件，形成2749个SpecBlock、151份现代OracleIR和35份legacy逻辑规格。逐条IR与在线话题、字段和函数的映射仍待固化。",
    },
    {
        "name": "测试输入覆盖\n四种类型",
        "baseline": "具备通用ROS消息变异和基础Campaign，尚未形成完整四维语义输入体系。",
        "requirement": "输入包含用户指令、传感器数据、控制参数和消息事件四类。",
        "status": "部分达到",
        "current": "部分达到。用户指令较强，控制参数中等，持续、频率、顺序、模式切换等消息事件部分支持；传感器目前主要用于Oracle观测，专用连续扰动及通用延迟、丢包、乱序仍待补强。",
    },
    {
        "name": "语义相关变异\n算子不少于5类",
        "baseline": "以字段级随机或确定性变异为主，行为时间语义和物理耦合有限。",
        "requirement": "语义相关的变异算子不少于5类。",
        "status": "达到：8类",
        "current": "达到：8类。已形成多轴协同、方向翻转、持续块、边界推进、反转、轨迹弧、单点极值和回环八类跨目标策略；随机仅作探索兜底，算子级独立贡献仍需消融。",
    },
    {
        "name": "偏轨判定与\n场景回溯",
        "baseline": "具备基础checker、rosbag和replay能力，但控制语义和证据归并不足。",
        "requirement": "支持控制语义偏轨判定与场景回溯验证。",
        "status": "仿真侧达到",
        "current": "仿真侧达到。五类现代目标已建立运行时Oracle，并形成rosbag、PX4 ULG、Gazebo独立真值、输入恢复、离线复算和回放证据链；Go2、ECO65-B实物重复执行和虚实差分尚未开展。",
    },
    {
        "name": "语义约束覆盖率\n相对提升≥20%",
        "baseline": "尚无统一的语义约束覆盖率定义、分母和同预算对照流程。",
        "requirement": "相较RoboFuzz，控制语义约束覆盖率提升不少于20%。",
        "status": "待形成正式证据",
        "current": "待形成正式证据。TB4组合优化中唯一输入签名629→842（+33.9%），最高单一输入签名重复次数156→21（下降86.5%）。该结果不是语义约束覆盖率，也不是单变量消融。",
    },
    {
        "name": "平台与漏洞\n成果指标",
        "baseline": "无本人可核实的实物验证闭环和CNVD/CNNVD正式编号基线。",
        "requirement": "至少2类典型平台、未公开漏洞不少于5个，并获得CNVD或CNNVD编号。",
        "status": "待形成正式证据",
        "current": "待形成正式证据。截至2026-09-17共提交34条公开Issue，其中22条Open、12条Closed。Issue不等于独立或确认漏洞；指定实物验证、未公开漏洞认定和正式编号尚未闭环。",
    },
]

TASKS = [
    {"id": 1, "title": "规范驱动的语义约束提取与行为 Oracle 构建"},
    {"id": 2, "title": "基于语义反馈的扰动输入生成与种子调度"},
    {"id": 3, "title": "控制语义偏轨判定、证据回溯与根因归并"},
]

TECH_SLIDES = [
    {
        "title": "三项重点技术任务与总体链路",
        "difficulty": "正确行为难定义、稀疏异常路径难激活、非崩溃偏轨难验证。",
        "asset": "01_三任务总体链路.svg",
        "method_lines": [
            "先从系统规范中明确机器人在不同状态下应当怎样响应，再生成带有控制语义的测试输入，最后结合仿真记录和回放结果确认异常是否稳定存在。",
            "把前述工作接成一个能够相互反馈的测试流程：验证阶段发现的问题可以反过来补充规则并调整后续输入，使测试不只是增加轮次，而是逐步逼近真正有价值的问题。",
        ],
        "boundary": "当前已形成跨目标仿真侧原型，不同目标成熟度不一；实物差分与正式漏洞编号为下一阶段。",
    },
    {
        "title": "任务一：规范驱动的语义约束提取与行为 Oracle 构建",
        "difficulty": "建模难：多源规范异构、跨模块约束隐含，难以形成可执行且可追溯的行为判据。",
        "asset": "02_规范到Oracle链路.svg",
        "method_lines": [
            "先按文件类型分别处理Launch、YAML、接口、源码和说明文档，把与控制行为有关的内容整理成能够追溯到原文的规范片段，再据此生成结构化约束。",
            "生成Oracle时，先找到与当前行为有关的规范，再检查字段、单位、适用状态和时间条件；对于依赖具体话题或状态机的规则，再结合目标系统补充上下文，避免把模型生成内容直接当作最终判断代码。",
        ],
        "boundary": "并非151份IR均已逐条在线执行；复杂约束仍需结合具体目标上下文。",
    },
    {
        "title": "任务二：基于语义反馈的扰动输入生成与种子调度",
        "difficulty": "激活难：高维状态空间稀疏，单字段随机变异难以触发时序和物理耦合型异常。",
        "asset": "03_语义反馈与种子调度.svg",
        "method_lines": [
            "测试输入不再局限于修改一条消息的数值，而是根据真实控制过程组织一段连续操作，让机器人经历保持、转向、接近边界和状态切换等变化，更容易到达随机变异很难触及的状态。",
            "每轮执行结束后，综合系统响应、任务结果和数据质量挑选更有价值的种子，同时限制相似种子反复入队或长期占用预算，使后续测试既能延续有效探索，也能及时转向新的行为区域。",
        ],
        "boundary": "组合优化结果，不等同于语义约束覆盖率；TB4数据不能作为指标5的正式完成证据。",
    },
    {
        "title": "任务三：控制语义偏轨判定、证据回溯与根因归并",
        "difficulty": "判定难：系统无崩溃、无错误码，异步多源观测和级联报警难以复现与归因。",
        "asset": "04_偏轨证据与根因归并.svg",
        "method_lines": [
            "发现偏轨后，同时保存测试输入、运行话题、仿真真值和任务结果，再用离线计算或回放确认异常能否再次出现，从而排除目标未就绪、数据缺失等环境因素。",
            "对于多次出现或彼此关联的异常，结合触发输入、发生时刻以及它们在控制链上的传递关系判断是否来自同一个原因，最终只保留证据完整、能够稳定复现的问题用于分析和上报。",
        ],
        "boundary": "当前完成仿真侧偏轨判定与回溯；实物验证、未公开漏洞认定和正式编号尚未闭环。",
    },
]

SLIDES = [
    {"kind": "toc", "active": 2},
    {"kind": "metrics"},
    {"kind": "toc", "active": 3},
    *[{"kind": "technical", **item} for item in TECH_SLIDES],
]

FOOTNOTE = (
    "统计截至2026-09-17；状态采用离散证据等级，不代表完成百分比。"
    "34 条 Issue 不等于 34 个漏洞；SpecBlock和IR数量也不等于全部在线执行。"
)


def all_visible_text() -> str:
    parts = TOC + [FOOTNOTE]
    for row in METRICS:
        parts.extend(str(value) for value in row.values())
    for slide in TECH_SLIDES:
        parts.extend([slide["title"], slide["difficulty"], slide["boundary"]])
        parts.extend(slide["method_lines"])
    parts.extend(["组合优化结果，不等同于语义约束覆盖率", "下一阶段｜尚未开展", "成熟度不一"])
    return "\n".join(parts)
```

- [ ] **Step 4: 运行内容测试并确认通过**

Run:

```bash
python3 -m unittest 人才项目/PPT_第2-3部分/tests/test_content.py -v
```

Expected: `Ran 6 tests`，结果为 `OK`。

- [ ] **Step 5: 提交内容清单和测试**

```bash
git add 人才项目/PPT_第2-3部分/tools/content.py 人才项目/PPT_第2-3部分/tests/test_content.py
git commit -m "test: freeze RoboFuzz PPT content claims"
```

---

### Task 2: 实现四张高信息密度 SVG/PNG 技术图

**Files:**
- Create: `人才项目/PPT_第2-3部分/tools/svg_diagrams.py`
- Create: `人才项目/PPT_第2-3部分/tools/generate_diagrams.py`
- Create: `人才项目/PPT_第2-3部分/tests/test_diagrams.py`
- Create: `人才项目/PPT_第2-3部分/assets/01_三任务总体链路.svg`
- Create: `人才项目/PPT_第2-3部分/assets/01_三任务总体链路.png`
- Create: `人才项目/PPT_第2-3部分/assets/02_规范到Oracle链路.svg`
- Create: `人才项目/PPT_第2-3部分/assets/02_规范到Oracle链路.png`
- Create: `人才项目/PPT_第2-3部分/assets/03_语义反馈与种子调度.svg`
- Create: `人才项目/PPT_第2-3部分/assets/03_语义反馈与种子调度.png`
- Create: `人才项目/PPT_第2-3部分/assets/04_偏轨证据与根因归并.svg`
- Create: `人才项目/PPT_第2-3部分/assets/04_偏轨证据与根因归并.png`

- [ ] **Step 1: 创建 SVG 结构测试**

在 `test_diagrams.py` 中写入：

```python
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
ASSETS = ROOT / "assets"
sys.path.insert(0, str(TOOLS))

import svg_diagrams


class DiagramTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        subprocess.run([sys.executable, str(TOOLS / "generate_diagrams.py")], check=True)

    def test_expected_assets_exist(self):
        for stem in svg_diagrams.OUTPUT_STEMS:
            self.assertTrue((ASSETS / f"{stem}.svg").is_file())
            self.assertTrue((ASSETS / f"{stem}.png").is_file())

    def test_svg_canvas_and_square_rectangles(self):
        ns = {"svg": "http://www.w3.org/2000/svg"}
        for stem in svg_diagrams.OUTPUT_STEMS:
            root = ET.parse(ASSETS / f"{stem}.svg").getroot()
            self.assertEqual(root.attrib["viewBox"], "0 0 2400 800")
            for rect in root.findall(".//svg:rect", ns):
                self.assertNotIn("rx", rect.attrib)
                self.assertNotIn("ry", rect.attrib)

    def test_text_is_large_enough(self):
        ns = {"svg": "http://www.w3.org/2000/svg"}
        for stem in svg_diagrams.OUTPUT_STEMS:
            root = ET.parse(ASSETS / f"{stem}.svg").getroot()
            sizes = [float(node.attrib["font-size"]) for node in root.findall(".//svg:text", ns)]
            self.assertGreaterEqual(min(sizes), 42.0)

    def test_all_polylines_are_orthogonal(self):
        ns = {"svg": "http://www.w3.org/2000/svg"}
        for stem in svg_diagrams.OUTPUT_STEMS:
            root = ET.parse(ASSETS / f"{stem}.svg").getroot()
            for line in root.findall(".//svg:polyline", ns):
                pts = [tuple(map(float, item.split(","))) for item in line.attrib["points"].split()]
                for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
                    self.assertTrue(x1 == x2 or y1 == y2, line.attrib["points"])

    def test_key_labels_are_present(self):
        expected = {
            "01_三任务总体链路": ("规范分块", "反馈调度", "候选证据包"),
            "02_规范到Oracle链路": ("SpecBlock", "自动化边界", "不可判定"),
            "03_语义反馈与种子调度": ("容量 ≤ 50", "边界压力 → 边界推进", "组合优化结果"),
            "04_偏轨证据与根因归并": ("不可判定", "去重异常签名", "非一一对应"),
        }
        for stem, labels in expected.items():
            text = (ASSETS / f"{stem}.svg").read_text(encoding="utf-8")
            for label in labels:
                self.assertIn(label, text)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行测试并确认因生成器不存在而失败**

Run:

```bash
python3 -m unittest 人才项目/PPT_第2-3部分/tests/test_diagrams.py -v
```

Expected: 导入 `svg_diagrams` 或调用 `generate_diagrams.py` 失败。

- [ ] **Step 3: 实现通用 SVG 画布和正交箭头约束**

`svg_diagrams.py` 的公共接口固定为：

```python
from __future__ import annotations

from dataclasses import dataclass, field
from html import escape
from pathlib import Path
from typing import Iterable

W, H = 2400, 800
FONT = "Microsoft YaHei, Noto Sans CJK SC, sans-serif"
COLORS = {
    "navy": "#002060",
    "blue": "#0070C0",
    "line": "#6592F5",
    "cyan": "#DDF4FF",
    "orange": "#EF962D",
    "orange_fill": "#FFF3E2",
    "green": "#289D64",
    "green_fill": "#EAF7EF",
    "red": "#C00000",
    "gray": "#F3F6F8",
    "gray_text": "#5F6B76",
    "black": "#111111",
    "white": "#FFFFFF",
}

OUTPUT_STEMS = (
    "01_三任务总体链路",
    "02_规范到Oracle链路",
    "03_语义反馈与种子调度",
    "04_偏轨证据与根因归并",
)


@dataclass
class SvgCanvas:
    items: list[str] = field(default_factory=list)

    def rect(self, x, y, w, h, fill="#FFFFFF", stroke="#002060", width=3, dash=None):
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        self.items.append(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="{width}"{dash_attr}/>'
        )

    def text(self, x, y, value, size=44, color="#111111", weight=600, anchor="middle"):
        self.items.append(
            f'<text x="{x}" y="{y}" font-family="{FONT}" font-size="{size}" '
            f'font-weight="{weight}" fill="{color}" text-anchor="{anchor}">{escape(value)}</text>'
        )

    def multiline(self, x, y, lines: Iterable[str], size=44, color="#111111", weight=600,
                  anchor="middle", gap=52):
        for index, line in enumerate(lines):
            self.text(x, y + index * gap, line, size=size, color=color, weight=weight, anchor=anchor)

    def polyline(self, points, color="#0070C0", width=5, dash=None, arrow=True):
        for first, second in zip(points, points[1:]):
            if first[0] != second[0] and first[1] != second[1]:
                raise ValueError(f"non-orthogonal segment: {first} -> {second}")
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        marker_ids = {
            COLORS["blue"]: "arrow-blue",
            COLORS["green"]: "arrow-green",
            COLORS["orange"]: "arrow-orange",
            COLORS["gray_text"]: "arrow-gray",
            COLORS["black"]: "arrow-black",
        }
        marker = f' marker-end="url(#{marker_ids.get(color, "arrow-blue")})"' if arrow else ""
        pts = " ".join(f"{x},{y}" for x, y in points)
        self.items.append(
            f'<polyline points="{pts}" fill="none" stroke="{color}" '
            f'stroke-width="{width}"{dash_attr}{marker}/>'
        )

    def render(self) -> str:
        body = "\n".join(self.items)
        markers = "".join(
            f'<marker id="arrow-{name}" markerWidth="12" markerHeight="12" refX="10" refY="6" '
            f'orient="auto"><path d="M0,0 L12,6 L0,12 z" fill="{color}"/></marker>'
            for name, color in (
                ("blue", COLORS["blue"]),
                ("green", COLORS["green"]),
                ("orange", COLORS["orange"]),
                ("gray", COLORS["gray_text"]),
                ("black", COLORS["black"]),
            )
        )
        return (
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">\n'
            f'<defs>{markers}</defs>\n'
            f'{body}\n</svg>\n'
        )


def box(canvas, x, y, w, h, title, lines=(), fill=None, stroke=None, dash=None,
        title_color=None, title_size=48, line_size=42, gap=48):
    fill = fill or COLORS["white"]
    stroke = stroke or COLORS["navy"]
    title_color = title_color or COLORS["navy"]
    canvas.rect(x, y, w, h, fill=fill, stroke=stroke, width=4, dash=dash)
    canvas.text(x + w / 2, y + 58, title, size=title_size, color=title_color, weight=700)
    if lines:
        canvas.multiline(x + w / 2, y + 115, lines, size=line_size, gap=gap)


def label(canvas, x, y, w, h, value, fill=None, stroke=None, color=None, dash=None, size=42):
    fill = fill or COLORS["white"]
    stroke = stroke or COLORS["navy"]
    color = color or COLORS["black"]
    canvas.rect(x, y, w, h, fill=fill, stroke=stroke, width=3, dash=dash)
    canvas.text(x + w / 2, y + h / 2 + size * 0.34, value, size=size, color=color, weight=600)


def draw_overview() -> str:
    c = SvgCanvas()
    box(c, 20, 80, 330, 165, "系统规范", ("文档 · 配置", "接口 · 源码"), fill=COLORS["cyan"])
    box(c, 20, 285, 330, 165, "目标系统", ("话题 · 参数", "状态机"), fill=COLORS["cyan"])
    box(c, 20, 490, 330, 165, "实验场景", ("平台 · 任务", "仿真环境"), fill=COLORS["cyan"])
    c.rect(390, 40, 1480, 680, fill=COLORS["gray"], stroke=COLORS["navy"], width=4)
    c.text(1130, 95, "规范驱动的语义模糊测试闭环", size=56, color=COLORS["navy"], weight=700)
    box(c, 430, 145, 420, 360, "任务一",
        ("定义正确行为", "规范分块", "结构化约束", "运行时 Oracle"), fill=COLORS["cyan"], gap=45)
    box(c, 920, 145, 420, 360, "任务二",
        ("激活异常路径", "行为种子", "语义变异", "反馈调度"), fill=COLORS["orange_fill"], stroke=COLORS["orange"], gap=45)
    box(c, 1410, 145, 420, 360, "任务三",
        ("验证行为偏轨", "在线判定", "复算回放", "根因归并"), fill=COLORS["green_fill"], stroke=COLORS["green"], gap=45)
    for y in (162, 367, 572):
        c.polyline([(350, y), (390, y)], color=COLORS["blue"])
    c.polyline([(850, 325), (920, 325)], color=COLORS["blue"])
    c.polyline([(1340, 325), (1410, 325)], color=COLORS["blue"])
    c.polyline([(1620, 510), (1620, 570), (650, 570), (650, 510)],
               color=COLORS["green"], dash="16 12")
    c.polyline([(1130, 570), (1130, 510)], color=COLORS["green"], dash="16 12")
    c.text(1130, 555, "补充规则 · 调整输入", size=42, color=COLORS["green"], weight=700)
    label(c, 430, 625, 1400, 62, "PX4　TurtleBot4　MoveIt2　SLAM　Nav2｜五类仿真目标已接入，成熟度不一",
          fill=COLORS["white"], stroke=COLORS["gray_text"], size=42)
    for index, text in enumerate(("可追溯规则", "高价值种子", "候选证据包", "公开沟通材料")):
        label(c, 1920, 80 + index * 150, 450, 105, text,
              fill=COLORS["green_fill"], stroke=COLORS["green"])
    c.polyline([(1870, 210), (1920, 210)], color=COLORS["green"])
    c.polyline([(1870, 360), (1920, 360)], color=COLORS["green"])
    c.polyline([(1870, 510), (1920, 510)], color=COLORS["green"])
    return c.render()


def draw_oracle_pipeline() -> str:
    c = SvgCanvas()
    c.text(850, 35, "当前工作区 · 五个现代目标", size=42, color=COLORS["gray_text"], weight=600)
    box(c, 20, 50, 300, 550, "① 多源规范", fill=COLORS["cyan"])
    for y, value in ((135, "文档 / 注释"), (225, "配置 / Launch"), (315, "ROS 接口"), (405, "源码 / 模型")):
        label(c, 48, y, 244, 65, value, size=42)
    c.text(170, 555, "425 个输入文件", size=46, color=COLORS["blue"], weight=700)
    box(c, 350, 50, 330, 550, "② 预处理",
        ("格式识别", "分块 / 归一", "保留来源"), fill=COLORS["cyan"])
    label(c, 382, 385, 266, 135, "SpecBlock", fill=COLORS["white"], stroke=COLORS["blue"], size=48)
    c.text(515, 555, "2749 个 SpecBlock", size=42, color=COLORS["blue"], weight=700)
    c.text(515, 595, "内容 · 路径 · 上下文", size=42, color=COLORS["blue"], weight=600)
    box(c, 710, 50, 370, 550, "③ Agent 生成",
        ("检索相关规范", "生成候选 IR", "结构校验", "语义规则检查"), fill=COLORS["cyan"])
    label(c, 782, 455, 226, 75, "OracleIR", fill=COLORS["white"], stroke=COLORS["blue"], size=44)
    c.text(895, 575, "151 份现代 IR", size=42, color=COLORS["blue"], weight=700)
    c.polyline([(1010, 320), (1040, 320), (1040, 215), (965, 215)], color=COLORS["blue"], dash="12 10")
    c.text(1010, 185, "失败则修订", size=42, color=COLORS["blue"], weight=600)
    c.polyline([(1105, 50), (1105, 600)], color=COLORS["orange"], dash="15 12", arrow=False)
    c.text(1105, 35, "自动化边界", size=42, color=COLORS["orange"], weight=700)
    box(c, 1130, 50, 570, 550, "④ 目标上下文化", fill=COLORS["orange_fill"], stroke=COLORS["orange"])
    for x, y, value in (
        (1160, 145, "话题 / 字段"), (1430, 145, "状态 / 时间窗"),
        (1160, 245, "容差 / 噪声"), (1430, 245, "质量门控"),
    ):
        label(c, x, y, 240, 75, value, fill=COLORS["white"], stroke=COLORS["orange"], size=42)
    label(c, 1240, 360, 350, 85, "运行时判断", fill=COLORS["white"], stroke=COLORS["orange"], size=46)
    for x, text, color in ((1160, "正常", COLORS["green"]), (1345, "偏轨", COLORS["red"]), (1530, "不可判定", COLORS["gray_text"])):
        label(c, x, 480, 150, 72, text, fill=COLORS["white"], stroke=color, color=color, size=42)
    box(c, 1750, 50, 630, 550, "OracleIR｜关键字段", fill=COLORS["gray"])
    fields = (("看什么", "观察量"), ("阈值从哪来", "参数 / 单位"), ("怎么算", "派生量"),
              ("何时算错", "断言"), ("何时生效", "作用域"), ("看多久", "时间窗"))
    for index, (question, answer) in enumerate(fields):
        col, row = index % 2, index // 2
        x, y = 1780 + col * 290, 125 + row * 140
        box(c, x, y, 260, 120, question, (answer,), fill=COLORS["white"], stroke=COLORS["navy"],
            title_size=42, line_size=42, gap=42)
    c.text(2065, 555, "来源证据：SpecBlock ID", size=42, color=COLORS["navy"], weight=700)
    c.polyline([(1008, 500), (1720, 500)], color=COLORS["gray_text"], dash="10 10", arrow=False)
    c.rect(20, 650, 2360, 120, fill=COLORS["green_fill"], stroke=COLORS["green"], width=3)
    trace = ((40, 310, "原始规范位置"), (390, 360, "SpecBlock"),
             (790, 300, "OracleIR"), (1130, 390, "运行时规则"), (1560, 500, "判定证据"))
    for x, w, value in trace:
        label(c, x, 675, w, 70, value, fill=COLORS["white"], stroke=COLORS["green"], size=42)
    for start, end in (((1560, 710), (1520, 710)), ((1130, 710), (1090, 710)),
                       ((790, 710), (750, 710)), ((390, 710), (350, 710))):
        c.polyline([start, end], color=COLORS["green"])
    return c.render()


def draw_seed_feedback() -> str:
    c = SvgCanvas()
    box(c, 20, 60, 300, 620, "多维输入与时间属性", fill=COLORS["cyan"])
    for index, value in enumerate(("用户指令", "控制参数", "消息事件", "传感数据*")):
        label(c, 55, 150 + index * 95, 230, 68, value, size=42)
    c.text(170, 560, "t₀ → t₁ → … → tₙ", size=42, color=COLORS["blue"], weight=700)
    c.multiline(170, 615, ("顺序 · 持续", "频率 · 模式"), size=42, color=COLORS["gray_text"], gap=45)
    c.rect(360, 40, 1460, 680, fill=COLORS["gray"], stroke=COLORS["navy"], width=4)
    box(c, 390, 85, 390, 285, "种子治理与选择",
        ("容量 ≤ 50", "单种子 ≤ 5 次", "近似去重 / 陈旧淘汰", "初始轮转 / 新鲜度"),
        fill=COLORS["cyan"], gap=42)
    box(c, 820, 85, 310, 285, "行为序列构造",
        ("消息序列", "持续时间", "顺序 / 模式"), fill=COLORS["cyan"])
    box(c, 1170, 85, 620, 285, "反馈自适应的语义变异",
        ("基础权重 + 反馈提升",), fill=COLORS["orange_fill"], stroke=COLORS["orange"])
    strategies = ("多轴", "翻转", "持续", "边界", "反转", "弧线", "极值", "回环")
    for index, value in enumerate(strategies):
        col, row = index % 4, index // 4
        label(c, 1195 + col * 145, 215 + row * 70, 125, 55, value,
              fill=COLORS["white"], stroke=COLORS["orange"], size=42)
    c.polyline([(780, 225), (820, 225)], color=COLORS["blue"])
    c.polyline([(1130, 225), (1170, 225)], color=COLORS["blue"])
    c.rect(390, 455, 1400, 205, fill=COLORS["green_fill"], stroke=COLORS["green"], width=4)
    segments = ((390, 350, "质量门控", ("就绪 · 有效", "任务可判定")),
                (740, 350, "语义反馈", ("运动 · 跟踪", "任务 · 质量")),
                (1090, 350, "新颖性", ("连续反馈", "错误签名")),
                (1440, 350, "队列治理", ("保留新反馈", "抑制重复")))
    for x, w, title, lines in segments:
        c.text(x + w / 2, 515, title, size=46, color=COLORS["green"], weight=700)
        c.multiline(x + w / 2, 570, lines, size=42, color=COLORS["black"], weight=600, gap=44)
    for x in (740, 1090, 1440):
        c.polyline([(x, 475), (x, 640)], color=COLORS["green"], arrow=False)
    c.polyline([(1440, 640), (1090, 640)], color=COLORS["green"])
    c.polyline([(1090, 640), (740, 640)], color=COLORS["green"])
    c.polyline([(740, 640), (390, 640)], color=COLORS["green"])
    c.polyline([(410, 455), (410, 405), (585, 405), (585, 370)], color=COLORS["green"], dash="16 12")
    c.polyline([(915, 455), (915, 405), (1450, 405), (1450, 370)], color=COLORS["orange"], dash="14 10")
    c.text(1300, 440, "策略映射：边界压力 → 边界推进", size=42, color=COLORS["orange"], weight=700)
    c.text(530, 700, "证据不足 / 基础设施异常 → 不入队", size=42, color=COLORS["gray_text"], weight=600)
    box(c, 1860, 60, 520, 315, "目标执行与观测",
        ("TB4 · PX4 · MoveIt2", "SLAM · Nav2", "控制状态 · 任务结果 · 运行质量"), fill=COLORS["cyan"])
    c.text(2120, 350, "按目标适配，成熟度不同", size=42, color=COLORS["gray_text"], weight=600)
    c.polyline([(1790, 225), (1860, 225)], color=COLORS["blue"])
    c.polyline([(2120, 375), (2120, 640), (1790, 640)], color=COLORS["green"])
    box(c, 1860, 420, 520, 300, "TB4｜组合优化结果",
        ("唯一输入签名", "629 → 842｜+33.9%", "单一输入最高重复", "156 → 21｜下降86.5%",
         "组合结果 ≠ 约束覆盖率"), fill=COLORS["green_fill"], stroke=COLORS["green"], gap=42)
    return c.render()


def draw_evidence_root_cause() -> str:
    c = SvgCanvas()
    box(c, 20, 30, 390, 610, "① 证据恢复",
        ("触发输入与参数", "Topic / Action / TF", "任务模式与状态",
         "rosbag / ULG", "仿真真值 / 任务结果", "按目标选用"), fill=COLORS["cyan"], gap=46)
    box(c, 440, 30, 330, 610, "② 质量门控",
        ("关键字段齐全", "时钟 / 样本对齐", "有效运动 / 有效任务", "基础设施异常", "可判定 → Oracle",
         "不可判定 ≠ 报警"), fill=COLORS["orange_fill"], stroke=COLORS["orange"], gap=46)
    box(c, 800, 30, 330, 610, "③ 在线 Oracle",
        ("状态前置", "窗口与持续性", "范围 / 时序约束", "跨话题一致性", "任务结果一致性", "偏轨标签",
         "报警帧 / 错误轮次"), fill=COLORS["orange_fill"], stroke=COLORS["orange"], gap=46)
    box(c, 1160, 30, 520, 610, "④ 复核与根因归并",
        ("离线复算", "独立回放", "恢复同一触发输入", "交叉证据验证", "成立｜边界｜证伪", "同输入 · 同时段 · 同控制链",
         "错误签名去重 → 级联合并", "候选根因族"), fill=COLORS["green_fill"], stroke=COLORS["green"], gap=43)
    for start, end in (((410, 335), (440, 335)), ((770, 335), (800, 335)), ((1130, 335), (1160, 335))):
        c.polyline([start, end], color=COLORS["blue"])
    c.text(2055, 30, "计数口径收敛", size=50, color=COLORS["navy"], weight=700)
    funnel = ((1740, 70, 630, "报警帧｜单时刻命中"),
              (1780, 155, 550, "错误轮次｜测试内命中"),
              (1820, 240, 470, "去重异常签名｜依目标定义"),
              (1860, 325, 390, "候选根因族"),
              (1900, 410, 310, "独立漏洞*"))
    for x, y, w, value in funnel:
        label(c, x, y, w, 64, value, fill=COLORS["gray"], stroke=COLORS["navy"], size=42)
    label(c, 1740, 475, 300, 58, "公开 Issue｜34条", fill=COLORS["cyan"], stroke=COLORS["blue"], size=42)
    label(c, 2070, 475, 300, 58, "22 Open / 12 Closed", fill=COLORS["cyan"], stroke=COLORS["blue"], size=42)
    c.text(2055, 545, "Issue 与漏洞非一一对应", size=42, color=COLORS["blue"], weight=700)
    c.text(2260, 455, "*尚未认定", size=42, color=COLORS["red"], weight=700)
    c.polyline([(2040, 514), (2110, 514), (2110, 357), (2250, 357)], color=COLORS["blue"], dash="10 10", arrow=False)
    c.rect(1730, 550, 650, 190, fill=COLORS["green_fill"], stroke=COLORS["green"], width=3)
    c.multiline(2055, 605, ("TB4：928轮 → 781签名", "→ 1条加速度路径候选链",
                              "MoveIt：224帧 → 1个候选族", "34条Issue ≠ 34个漏洞"),
                size=42, color=COLORS["black"], gap=42)
    c.rect(760, 660, 900, 110, fill=COLORS["orange_fill"], stroke=COLORS["orange"], width=4, dash="15 12")
    c.text(1210, 705, "下一阶段｜尚未开展", size=46, color=COLORS["orange"], weight=700)
    c.text(1210, 755, "同一安全种子 → Go2 / ECO65-B → 虚实事件对齐", size=42, color=COLORS["black"], weight=600)
    c.polyline([(1420, 640), (1420, 660)], color=COLORS["orange"], dash="15 12")
    return c.render()
```

四个 `draw_*()` 函数必须使用以下一级分区坐标；子框在各自分区内部保持至少 22 个 viewBox 单位间距：

| 图 | 一级区域坐标 `(x,y,w,h)` |
|---|---|
| 总览 | 输入 `(20,80,330,600)`；系统闭环 `(390,40,1480,680)`；资产 `(1920,80,450,600)` |
| 任务一 | 规范 `(20,50,300,550)`；预处理 `(350,50,330,550)`；Agent `(710,50,370,550)`；上下文化 `(1130,50,570,550)`；IR 放大 `(1750,50,630,550)`；追溯带 `(20,650,2360,120)` |
| 任务二 | 输入 `(20,60,300,620)`；闭环 `(360,40,1460,680)`；执行与结果 `(1860,60,520,660)` |
| 任务三 | 判定主链 `(20,30,1660,610)`；口径漏斗 `(1730,30,650,500)`；证据带 `(1730,550,650,190)`；下一阶段 `(760,660,900,110)` |

上面的四个完整函数即为实现基线。图中文字严格采用设计说明第 6.1—6.4 节的最终短字段。执行时还必须检查：

- 总览反馈线在平台条上方留独立通道；
- 任务一反向追溯必须为 `原始来源 ← SpecBlock ← OracleIR ← 运行时规则 ← 判定证据`；
- 任务二只保留一个策略映射例子 `边界压力 → 边界推进`；
- 任务三 Issue 与根因/漏洞使用点线并标 `非一一对应`；
- Go2/ECO65-B 分支为橙色虚线且写明 `下一阶段｜尚未开展`。

- [ ] **Step 4: 实现生成入口和 PNG 转换**

`generate_diagrams.py` 写入：

```python
from pathlib import Path
import subprocess

from svg_diagrams import (
    OUTPUT_STEMS,
    draw_overview,
    draw_oracle_pipeline,
    draw_seed_feedback,
    draw_evidence_root_cause,
)

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets"
BUILDERS = (
    draw_overview,
    draw_oracle_pipeline,
    draw_seed_feedback,
    draw_evidence_root_cause,
)


def main():
    ASSETS.mkdir(parents=True, exist_ok=True)
    for stem, builder in zip(OUTPUT_STEMS, BUILDERS, strict=True):
        svg_path = ASSETS / f"{stem}.svg"
        png_path = ASSETS / f"{stem}.png"
        svg_path.write_text(builder(), encoding="utf-8")
        subprocess.run(
            ["rsvg-convert", "-w", "2400", "-h", "800", "-o", str(png_path), str(svg_path)],
            check=True,
        )


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: 生成图并运行结构测试**

Run:

```bash
python3 人才项目/PPT_第2-3部分/tools/generate_diagrams.py
python3 -m unittest 人才项目/PPT_第2-3部分/tests/test_diagrams.py -v
```

Expected: 四个 SVG 和四个 PNG 均生成，`Ran 5 tests`，结果为 `OK`。

- [ ] **Step 6: 生成四图预览并进行第一次人工目视检查**

Run:

```bash
montage 人才项目/PPT_第2-3部分/assets/0*.png -tile 1x4 -geometry 1200x400+12+12 /tmp/robofuzz-progress-diagrams.png
```

使用图像查看工具检查：无圆角、无文本截断、无箭头穿框、无重叠；若某字段低于 15 pt 才能放下，按设计说明删除次要字段，不缩小字号。

- [ ] **Step 7: 提交绘图代码、测试和八个资产**

```bash
git add 人才项目/PPT_第2-3部分/tools/svg_diagrams.py 人才项目/PPT_第2-3部分/tools/generate_diagrams.py 人才项目/PPT_第2-3部分/tests/test_diagrams.py 人才项目/PPT_第2-3部分/assets
git commit -m "feat: add high-density RoboFuzz PPT diagrams"
```

---

### Task 3: 实现 LibreOffice UNO 编辑原语并验证模板副本安全

**Files:**
- Create: `人才项目/PPT_第2-3部分/tools/uno_ppt.py`
- Create: `人才项目/PPT_第2-3部分/tests/test_ppt_build.py`

- [ ] **Step 1: 创建模板安全和七页输出的集成测试**

`test_ppt_build.py` 先写入：

```python
import hashlib
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

import uno_ppt

SOURCE = REPO / "人才项目" / "拔尖人才项目阶段进展汇报.pptx"


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class UnoPptTests(unittest.TestCase):
    def test_editing_copy_does_not_change_source(self):
        before = sha256(SOURCE)
        with tempfile.TemporaryDirectory() as tmp:
            copy_path = Path(tmp) / "copy.pptx"
            shutil.copy2(SOURCE, copy_path)
            with uno_ppt.ImpressSession() as session:
                doc = session.open(copy_path)
                self.assertEqual(doc.getDrawPages().getCount(), 8)
                doc.getDrawPages().remove(doc.getDrawPages().getByIndex(0))
                session.save_and_close(doc)
            with uno_ppt.ImpressSession() as session:
                doc = session.open(copy_path)
                self.assertEqual(doc.getDrawPages().getCount(), 7)
                session.close(doc)
        self.assertEqual(before, sha256(SOURCE))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 运行测试并确认因 `uno_ppt` 不存在而失败**

Run:

```bash
PYTHONPATH=/usr/lib/python3/dist-packages python3 -m unittest 人才项目/PPT_第2-3部分/tests/test_ppt_build.py -v
```

Expected: `ModuleNotFoundError: No module named 'uno_ppt'`。

- [ ] **Step 3: 实现隔离的 LibreOffice 会话**

`uno_ppt.py` 必须提供以下会话类；使用临时用户配置，避免连接或干扰用户当前打开的 LibreOffice：

```python
from __future__ import annotations

import shutil
import socket
import subprocess
import tempfile
import time
from pathlib import Path

import uno
from com.sun.star.beans import PropertyValue
from com.sun.star.awt import Point, Size


def prop(name, value):
    item = PropertyValue()
    item.Name = name
    item.Value = value
    return item


def file_url(path):
    return uno.systemPathToFileUrl(str(Path(path).resolve()))


def cm(value):
    return int(round(value * 1000))


class ImpressSession:
    def __init__(self):
        self.profile = None
        self.process = None
        self.context = None
        self.desktop = None

    def __enter__(self):
        self.profile = Path(tempfile.mkdtemp(prefix="robofuzz-lo-profile-"))
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        accept = f"socket,host=127.0.0.1,port={port};urp;StarOffice.ComponentContext"
        self.process = subprocess.Popen(
            [
                "soffice", "--headless", "--nologo", "--nodefault", "--norestore",
                "--nofirststartwizard", f"-env:UserInstallation={file_url(self.profile)}",
                f"--accept={accept}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        local = uno.getComponentContext()
        resolver = local.ServiceManager.createInstanceWithContext(
            "com.sun.star.bridge.UnoUrlResolver", local
        )
        last_error = None
        for _ in range(80):
            try:
                self.context = resolver.resolve(
                    f"uno:socket,host=127.0.0.1,port={port};urp;StarOffice.ComponentContext"
                )
                break
            except Exception as error:
                last_error = error
                time.sleep(0.1)
        if self.context is None:
            raise RuntimeError("cannot connect to isolated LibreOffice") from last_error
        self.desktop = self.context.ServiceManager.createInstanceWithContext(
            "com.sun.star.frame.Desktop", self.context
        )
        return self

    def open(self, path):
        return self.desktop.loadComponentFromURL(
            file_url(path), "_blank", 0, (prop("Hidden", True), prop("ReadOnly", False))
        )

    def save_and_close(self, doc):
        doc.store()
        doc.close(True)

    def close(self, doc):
        doc.close(True)

    def __exit__(self, exc_type, exc, tb):
        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        if self.profile is not None:
            shutil.rmtree(self.profile, ignore_errors=True)
```

- [ ] **Step 4: 增加原生形状、表格和 SVG 插入辅助函数**

同一文件继续实现以下公共接口：

```python
def clear_page(page):
    for index in range(page.getCount() - 1, -1, -1):
        page.remove(page.getByIndex(index))


def add_text(doc, page, x, y, w, h, text, font_size, color=0x000000,
             bold=False, align=0, valign=0, border=None, fill=None, dash=False):
    shape = doc.createInstance("com.sun.star.drawing.TextShape")
    shape.Position = Point(cm(x), cm(y))
    shape.Size = Size(cm(w), cm(h))
    page.add(shape)
    shape.String = text
    shape.CharFontName = "Microsoft YaHei"
    shape.CharHeight = float(font_size)
    shape.CharWeight = 150.0 if bold else 100.0
    shape.CharColor = color
    shape.ParaAdjust = align
    shape.TextVerticalAdjust = valign
    shape.TextAutoGrowHeight = False
    if fill is None:
        shape.FillStyle = 0
    else:
        shape.FillStyle = 1
        shape.FillColor = fill
    if border is None:
        shape.LineStyle = 0
    else:
        shape.LineStyle = 2 if dash else 1
        shape.LineColor = border
        shape.LineWidth = 70
    return shape


def color_prefix(shape, prefix, color=0xC00000):
    if not shape.String.startswith(prefix):
        raise ValueError(f"text does not start with expected prefix: {prefix}")
    cursor = shape.createTextCursor()
    cursor.gotoStart(False)
    cursor.goRight(len(prefix), True)
    cursor.CharColor = color
    cursor.CharWeight = 150.0


def add_svg(session, doc, page, path, x, y, w, h):
    provider = session.context.ServiceManager.createInstanceWithContext(
        "com.sun.star.graphic.GraphicProvider", session.context
    )
    graphic = provider.queryGraphic((prop("URL", file_url(path)),))
    shape = doc.createInstance("com.sun.star.drawing.GraphicObjectShape")
    shape.Position = Point(cm(x), cm(y))
    shape.Size = Size(cm(w), cm(h))
    shape.Graphic = graphic
    page.add(shape)
    return shape


def make_table(doc, page, x, y, w, h, columns, rows):
    shape = doc.createInstance("com.sun.star.drawing.TableShape")
    shape.Position = Point(cm(x), cm(y))
    shape.Size = Size(cm(w), cm(h))
    page.add(shape)
    table = shape.Model
    table.initialize(columns, rows)
    return shape, table


def border_line(color=0x000000, width=35):
    line = uno.createUnoStruct("com.sun.star.table.BorderLine2")
    line.Color = color
    line.InnerLineWidth = 0
    line.OuterLineWidth = width
    line.LineDistance = 0
    line.LineWidth = width
    return line


def set_cell(cell, text, font_size=11.5, color=0x000000, bold=False,
             fill=0xFFFFFF, align=0, valign=0):
    cell.String = text
    cell.CharFontName = "Microsoft YaHei"
    cell.CharHeight = float(font_size)
    cell.CharWeight = 150.0 if bold else 100.0
    cell.CharColor = color
    cell.FillColor = fill
    cell.ParaAdjust = align
    cell.TextVerticalAdjust = valign
    line = border_line()
    cell.TopBorder = line
    cell.BottomBorder = line
    cell.LeftBorder = line
    cell.RightBorder = line
```

构建表格时，通过 `table.getColumns().getByIndex(i).Width` 和 `table.getRows().getByIndex(i).Height` 设置尺寸，并通过 `set_cell()` 统一应用文字、填充和 1 pt 黑色网格。

- [ ] **Step 5: 运行 UNO 集成测试并确认通过**

Run:

```bash
PYTHONPATH=/usr/lib/python3/dist-packages python3 -m unittest 人才项目/PPT_第2-3部分/tests/test_ppt_build.py -v
```

Expected: 输出副本重新打开后为七页，源模板哈希不变，结果为 `OK`。

- [ ] **Step 6: 提交 UNO 编辑原语和集成测试**

```bash
git add 人才项目/PPT_第2-3部分/tools/uno_ppt.py 人才项目/PPT_第2-3部分/tests/test_ppt_build.py
git commit -m "feat: add isolated LibreOffice PPT editing helpers"
```

---

### Task 4: 构建七页 PPTX

**Files:**
- Create: `人才项目/PPT_第2-3部分/tools/build_ppt.py`
- Modify: `人才项目/PPT_第2-3部分/tests/test_ppt_build.py`
- Create: `人才项目/PPT_第2-3部分/拔尖人才项目阶段进展汇报_RoboFuzz_第2-3部分.pptx`

- [ ] **Step 1: 扩展集成测试，固定最终页序和关键标题**

在 `test_ppt_build.py` 增加测试，调用构建脚本后通过解压 `ppt/presentation.xml` 检查七页，并从所有 `ppt/slides/slide*.xml` 提取文本：

```python
import re
import subprocess
import xml.etree.ElementTree as ET
import zipfile


def ppt_text(path):
    chunks = []
    with zipfile.ZipFile(path) as archive:
        for name in sorted(n for n in archive.namelist() if re.fullmatch(r"ppt/slides/slide\d+\.xml", n)):
            root = ET.fromstring(archive.read(name))
            chunks.extend(node.text or "" for node in root.iter() if node.tag.endswith("}t"))
    return "\n".join(chunks)


def presentation_page_count(path):
    ns = {"p": "http://schemas.openxmlformats.org/presentationml/2006/main"}
    with zipfile.ZipFile(path) as archive:
        root = ET.fromstring(archive.read("ppt/presentation.xml"))
    return len(root.findall(".//p:sldIdLst/p:sldId", ns))


class FinalBuildTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        subprocess.run(
            [sys.executable, str(TOOLS / "generate_diagrams.py")], check=True
        )
        subprocess.run(
            [sys.executable, str(TOOLS / "build_ppt.py")], check=True,
            env={**dict(__import__("os").environ), "PYTHONPATH": "/usr/lib/python3/dist-packages"},
        )

    def test_final_presentation_has_seven_pages(self):
        output = ROOT / "拔尖人才项目阶段进展汇报_RoboFuzz_第2-3部分.pptx"
        self.assertEqual(presentation_page_count(output), 7)

    def test_final_titles_and_boundaries_exist(self):
        output = ROOT / "拔尖人才项目阶段进展汇报_RoboFuzz_第2-3部分.pptx"
        text = ppt_text(output)
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
        output = ROOT / "拔尖人才项目阶段进展汇报_RoboFuzz_第2-3部分.pptx"
        text = ppt_text(output)
        for phrase in ("Peach", "语义流图", "协议代码测试并行执行", "留到绩效自评价报告"):
            self.assertNotIn(phrase, text)
```

- [ ] **Step 2: 运行扩展测试并确认因构建脚本不存在而失败**

Run:

```bash
PYTHONPATH=/usr/lib/python3/dist-packages python3 -m unittest 人才项目/PPT_第2-3部分/tests/test_ppt_build.py -v
```

Expected: 无法打开 `build_ppt.py` 或最终 PPTX，测试失败。

- [ ] **Step 3: 实现目录页文字替换**

`build_ppt.py` 的入口固定为：

```python
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
```

对保留下来的两个目录页执行：

```python
replacements = {
    "课题概要与研究内容": "课题概况与研究内容",
    "年度进展与重要成果": "技术细节",
}
replace_text(pages.getByIndex(0), replacements)
replace_text(pages.getByIndex(2), replacements)
```

不要重建目录页背景或编号块；原模板第 2 页已经突出 02，第 4 页已经突出 03。

- [ ] **Step 4: 重建四列表格页**

清空成品第 2 页的所有幻灯片级对象，保留母版背景和标题下横线，然后创建：

- 标题：`任务与指标完成情况`，`x=1.86,y=1.29,w=29.5,h=1.80`，32 pt；
- 青色横幅：`x=0.50,y=3.32,w=32.87,h=1.00`，文字为 `课题：具身智能系统控制逻辑漏洞挖掘技术研究`，18 pt；
- 原生表格：`x=0.50,y=4.32,w=32.87,h=13.35`，4 列 7 行；
- 页脚：`x=0.75,y=17.78,w=32.2,h=0.72`，11 pt 深灰。

四列宽度固定为：

```python
column_widths_cm = [4.27, 6.57, 7.23, 14.80]
```

七行高度固定为：

```python
row_heights_cm = [1.20, 2.30, 2.18, 1.62, 2.05, 2.00, 2.00]
```

表头为：

```python
["指标名称", "原始基线 / 立项状态", "开题完成要求", "当前指标状态"]
```

主体内容来自 `content.METRICS`；最后一列使用深红 `#C00000`，其余正文使用黑色。表头 14 pt 加粗，正文 11.5 pt；第一列居中，其余三列左对齐。若 LibreOffice 导出后表格溢出，只允许将正文降至 11 pt，不能删去边界说明。

`build_metrics_slide()` 的实现固定为：

```python
def build_metrics_slide(doc, page):
    clear_page(page)
    add_text(doc, page, 1.86, 1.29, 29.5, 1.80, "任务与指标完成情况", 32, bold=True)
    add_text(
        doc, page, 0.50, 3.32, 32.87, 1.00,
        "课题：具身智能系统控制逻辑漏洞挖掘技术研究",
        18, bold=True, align=3, valign=1, border=0x000000, fill=0x00B0F0,
    )
    _, table = make_table(doc, page, 0.50, 4.32, 32.87, 13.35, 4, 7)
    column_widths_cm = [4.27, 6.57, 7.23, 14.80]
    row_heights_cm = [1.20, 2.30, 2.18, 1.62, 2.05, 2.00, 2.00]
    for index, width in enumerate(column_widths_cm):
        table.getColumns().getByIndex(index).Width = cm(width)
    for index, height in enumerate(row_heights_cm):
        table.getRows().getByIndex(index).Height = cm(height)
    headers = ["指标名称", "原始基线 / 立项状态", "开题完成要求", "当前指标状态"]
    for col, value in enumerate(headers):
        set_cell(
            table.getCellByPosition(col, 0), value, font_size=14,
            color=0xC00000 if col == 3 else 0x000000,
            bold=True, fill=0xFFFFFF, align=3, valign=1,
        )
    for row_index, metric in enumerate(content.METRICS, start=1):
        values = (metric["name"], metric["baseline"], metric["requirement"], metric["current"])
        for col, value in enumerate(values):
            set_cell(
                table.getCellByPosition(col, row_index), value,
                font_size=11.5,
                color=0xC00000 if col == 3 else 0x000000,
                bold=(col == 0), fill=0xFFFFFF,
                align=3 if col == 0 else 0, valign=1,
            )
    add_text(doc, page, 0.75, 17.78, 32.2, 0.72, content.FOOTNOTE, 11,
             color=0x5F6B76, align=3, valign=1)
```

- [ ] **Step 5: 重建四张技术内容页**

对成品第 4—7 页逐页执行 `clear_page()`，然后用相同坐标重建：

```text
标题：       x=1.86, y=1.29, w=29.8, h=1.80
难点标签：   x=1.88, y=3.65, w=4.23, h=1.66
难点正文：   x=6.10, y=3.65, w=25.43, h=1.66
中央 SVG：   x=1.60, y=5.42, w=30.50, h=10.17
方法标签：   x=1.88, y=15.94, w=3.51, h=2.65
方法第一行： x=5.58, y=15.94, w=25.95, h=1.18
方法第二行： x=5.58, y=17.15, w=25.95, h=1.18
边界说明：   x=5.58, y=18.28, w=25.95, h=0.48
```

样式固定为：

- 标题：32 pt，长标题允许 30 pt；
- “主要难点”标签：24 pt、深蓝、2 pt 实线框；
- 难点正文：22—24 pt、2 pt 深蓝虚线框；
- “方法 / 创新”标签：24 pt、深蓝、2 pt 实线框；
- 两条自然语言方法：18—20 pt、2 pt 深蓝虚线框；
- 边界说明：12.5—13.5 pt、深灰，无边框；
- 中央图优先插入 SVG；只有 SVG 在 LibreOffice 中出现错位时才改插同名 PNG。

方法文字必须直接读取 `content.TECH_SLIDES`，不能在构建脚本内改写或生成口号。

`build_technical_slide()` 的实现固定为：

```python
def build_technical_slide(session, doc, page, spec):
    clear_page(page)
    title_size = 30 if len(spec["title"]) > 28 else 32
    add_text(doc, page, 1.86, 1.29, 29.8, 1.80, spec["title"], title_size, bold=True)
    add_text(
        doc, page, 1.88, 3.65, 4.23, 1.66, "主要难点", 24,
        color=0x002060, bold=True, align=3, valign=1, border=0x2D2D8A, fill=0xFFFFFF,
    )
    difficulty = add_text(
        doc, page, 6.10, 3.65, 25.43, 1.66, f"□ {spec['difficulty']}", 23,
        bold=True, align=0, valign=1, border=0x002060, fill=0xFFFFFF, dash=True,
    )
    prefix = f"□ {spec['difficulty'].split('：', 1)[0]}：" if "：" in spec["difficulty"] else "□ "
    color_prefix(difficulty, prefix)
    add_svg(session, doc, page, ASSETS / spec["asset"], 1.60, 5.42, 30.50, 10.17)
    add_text(
        doc, page, 1.88, 15.94, 3.51, 2.65, "方 法\n创 新", 24,
        color=0x002060, bold=True, align=3, valign=1, border=0x2D2D8A, fill=0xFFFFFF,
    )
    method_size = 18 if max(map(len, spec["method_lines"])) > 62 else 20
    add_text(
        doc, page, 5.58, 15.94, 25.95, 1.18, f"□ {spec['method_lines'][0]}", method_size,
        bold=True, align=0, valign=1, border=0x002060, fill=0xFFFFFF, dash=True,
    )
    add_text(
        doc, page, 5.58, 17.15, 25.95, 1.08, f"□ {spec['method_lines'][1]}", method_size,
        bold=True, align=0, valign=1, border=0x002060, fill=0xFFFFFF, dash=True,
    )
    add_text(
        doc, page, 5.58, 18.26, 25.95, 0.48, spec["boundary"], 12.5,
        color=0x5F6B76, align=0, valign=1,
    )
```

- [ ] **Step 6: 完成构建入口和源模板保护**

`main()` 的执行顺序固定为：

```python
def main():
    ROOT.mkdir(parents=True, exist_ok=True)
    before = sha256(SOURCE)
    shutil.copy2(SOURCE, OUTPUT)
    with ImpressSession() as session:
        doc = session.open(OUTPUT)
        pages = doc.getDrawPages()
        if pages.getCount() != 8:
            raise RuntimeError(f"expected 8 template slides, got {pages.getCount()}")
        pages.remove(pages.getByIndex(0))
        replace_text(pages.getByIndex(0), {
            "课题概要与研究内容": "课题概况与研究内容",
            "年度进展与重要成果": "技术细节",
        })
        build_metrics_slide(doc, pages.getByIndex(1))
        replace_text(pages.getByIndex(2), {
            "课题概要与研究内容": "课题概况与研究内容",
            "年度进展与重要成果": "技术细节",
        })
        for offset, spec in enumerate(content.TECH_SLIDES, start=3):
            build_technical_slide(session, doc, pages.getByIndex(offset), spec)
        session.save_and_close(doc)
    after = sha256(SOURCE)
    if before != after:
        raise RuntimeError("source template changed during build")


if __name__ == "__main__":
    main()
```

- [ ] **Step 7: 运行完整构建测试**

Run:

```bash
python3 人才项目/PPT_第2-3部分/tools/generate_diagrams.py
PYTHONPATH=/usr/lib/python3/dist-packages python3 人才项目/PPT_第2-3部分/tools/build_ppt.py
PYTHONPATH=/usr/lib/python3/dist-packages python3 -m unittest 人才项目/PPT_第2-3部分/tests/test_ppt_build.py -v
```

Expected: 输出 PPTX 存在、页数为 7、指定标题均存在、旧示例文本不存在、源模板哈希不变。

- [ ] **Step 8: 提交构建脚本、测试和 PPTX**

```bash
git add 人才项目/PPT_第2-3部分/tools/build_ppt.py 人才项目/PPT_第2-3部分/tests/test_ppt_build.py 人才项目/PPT_第2-3部分/拔尖人才项目阶段进展汇报_RoboFuzz_第2-3部分.pptx
git commit -m "feat: build RoboFuzz progress PPT sections 2 and 3"
```

---

### Task 5: 生成内容说明、渲染预览和自动验收脚本

**Files:**
- Create: `人才项目/PPT_第2-3部分/tools/verify_artifacts.py`
- Create: `人才项目/PPT_第2-3部分/PPT_CONTENT_NOTES.md`
- Create: `人才项目/PPT_第2-3部分/PPT_第2-3部分_预览.png`

- [ ] **Step 1: 实现自动验收脚本**

`verify_artifacts.py` 必须完成以下检查：

```python
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


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def page_count(path):
    ns = {"p": "http://schemas.openxmlformats.org/presentationml/2006/main"}
    with zipfile.ZipFile(path) as archive:
        root = ET.fromstring(archive.read("ppt/presentation.xml"))
    return len(root.findall(".//p:sldIdLst/p:sldId", ns))


def ppt_text(path):
    values = []
    with zipfile.ZipFile(path) as archive:
        slide_names = sorted(
            (n for n in archive.namelist() if re.fullmatch(r"ppt/slides/slide\d+\.xml", n)),
            key=lambda n: int(re.search(r"\d+", n).group()),
        )
        for name in slide_names:
            root = ET.fromstring(archive.read(name))
            values.extend(node.text or "" for node in root.iter() if node.tag.endswith("}t"))
    return "\n".join(values)


def render_preview():
    render_dir = Path("/tmp/robofuzz-progress-ppt-render")
    if render_dir.exists():
        shutil.rmtree(render_dir)
    render_dir.mkdir(parents=True)
    profile = Path(tempfile.mkdtemp(prefix="robofuzz-lo-render-"))
    try:
        subprocess.run(
            [
                "soffice", "--headless", "--nologo", "--norestore",
                f"-env:UserInstallation={profile.as_uri()}",
                "--convert-to", "pdf", "--outdir", str(render_dir), str(OUTPUT),
            ],
            check=True,
        )
    finally:
        shutil.rmtree(profile, ignore_errors=True)
    pdf = render_dir / f"{OUTPUT.stem}.pdf"
    subprocess.run(["pdftoppm", "-png", "-r", "150", str(pdf), str(render_dir / "slide")], check=True)
    pages = sorted(render_dir.glob("slide-*.png"))
    if len(pages) != 7:
        raise RuntimeError(f"expected 7 rendered pages, got {len(pages)}")
    subprocess.run(
        ["montage", *map(str, pages), "-tile", "2x4", "-geometry", "960x540+12+12", str(PREVIEW)],
        check=True,
    )
    return pdf


def main():
    if page_count(OUTPUT) != 7:
        raise RuntimeError("PPTX page count is not 7")
    text = ppt_text(OUTPUT)
    for forbidden in ("Peach", "语义流图", "协议代码测试并行执行", "留到绩效自评价报告"):
        if forbidden in text:
            raise RuntimeError(f"forbidden template text remains: {forbidden}")
    for required in ("34 条 Issue 不等于 34 个漏洞", "组合优化结果，不等同于语义约束覆盖率"):
        if required not in text:
            raise RuntimeError(f"required boundary missing: {required}")
    for stem in (
        "01_三任务总体链路", "02_规范到Oracle链路",
        "03_语义反馈与种子调度", "04_偏轨证据与根因归并",
    ):
        for suffix in (".svg", ".png"):
            if not (ROOT / "assets" / f"{stem}{suffix}").is_file():
                raise RuntimeError(f"missing asset: {stem}{suffix}")
    pdf = render_preview()
    info = subprocess.check_output(["pdfinfo", str(pdf)], text=True)
    if "Pages:           7" not in info and not re.search(r"Pages:\s+7\b", info):
        raise RuntimeError("rendered PDF is not 7 pages")
    print(f"source_sha256={sha256(SOURCE)}")
    print(f"output_sha256={sha256(OUTPUT)}")
    print(f"preview={PREVIEW}")


if __name__ == "__main__":
    main()
```

构建脚本首次运行前额外记录一次源模板哈希到终端；验收脚本结束后再次输出源哈希，人工比较必须相同。

- [ ] **Step 2: 编写内容说明文档**

`PPT_CONTENT_NOTES.md` 必须逐页记录：

1. 成品页码和来源模板页码；
2. 该页标题、主要难点、方法创新和边界说明；
3. 图中每个数字的定义；
4. 不能在讲解时替换的口径，包括：
   - 151 是现代 OracleIR，不是全部在线执行的 Oracle；
   - 2749 是 SpecBlock，不是 2749 条已执行约束；
   - 629→842 是唯一输入签名增长，不是约束覆盖率；
   - 156→21 是最高单一输入签名重复次数下降；
   - 928、781、224 分别属于不同统计单位，不能相加；
   - 34 是公开 Issue 数，不是独立漏洞数；
   - Go2/ECO65-B 和虚实差分为下一阶段。
5. Windows PowerPoint 最终检查项：微软雅黑替换、表格行高、SVG 字体、标题是否换行。

- [ ] **Step 3: 运行自动验收并生成七页预览**

Run:

```bash
PYTHONPATH=/usr/lib/python3/dist-packages python3 人才项目/PPT_第2-3部分/tools/verify_artifacts.py
```

Expected:

- PDF 渲染为 7 页；
- `PPT_第2-3部分_预览.png` 生成；
- 终端输出源 PPTX、成品 PPTX 的 SHA-256；
- 无旧项目文字，无越界成果表述。

- [ ] **Step 4: 提交验收脚本、内容说明和预览图**

```bash
git add 人才项目/PPT_第2-3部分/tools/verify_artifacts.py 人才项目/PPT_第2-3部分/PPT_CONTENT_NOTES.md 人才项目/PPT_第2-3部分/PPT_第2-3部分_预览.png
git commit -m "docs: add RoboFuzz PPT notes and rendered preview"
```

---

### Task 6: 完整视觉复核与最终交付检查

**Files:**
- Modify: `人才项目/PPT_第2-3部分/tools/svg_diagrams.py`（仅在目视检查发现图形问题时）
- Modify: `人才项目/PPT_第2-3部分/tools/build_ppt.py`（仅在目视检查发现页面问题时）
- Regenerate: `人才项目/PPT_第2-3部分/assets/*.svg`
- Regenerate: `人才项目/PPT_第2-3部分/assets/*.png`
- Regenerate: `人才项目/PPT_第2-3部分/拔尖人才项目阶段进展汇报_RoboFuzz_第2-3部分.pptx`
- Regenerate: `人才项目/PPT_第2-3部分/PPT_第2-3部分_预览.png`

- [ ] **Step 1: 运行全部测试和构建命令**

```bash
python3 -m unittest discover -s 人才项目/PPT_第2-3部分/tests -v
python3 人才项目/PPT_第2-3部分/tools/generate_diagrams.py
PYTHONPATH=/usr/lib/python3/dist-packages python3 人才项目/PPT_第2-3部分/tools/build_ppt.py
PYTHONPATH=/usr/lib/python3/dist-packages python3 人才项目/PPT_第2-3部分/tools/verify_artifacts.py
```

Expected: 所有单元测试和集成测试通过，PPTX/PDF 均为 7 页，八个图形资产和总览图齐全。

- [ ] **Step 2: 使用总览图逐页进行视觉检查**

必须逐项确认：

- 第 1、3 页目录分别突出 02、03；
- 第 2 页没有第五列，最后一列标题为“当前指标状态”；
- 表格没有截字、行高溢出或低于 11 pt 的正文；
- 第 4—7 页标题、难点带、中央图、方法创新带互不覆盖；
- 四张中央图均为直角框；
- 总览反馈线不穿平台条；
- 任务一追溯箭头从判定证据返回原始来源；
- 任务二上层三个模块和下层反馈带方向正确；
- 任务三“不可判定”不进入异常计数，Issue 关系为点线且标注“非一一对应”；
- 下一阶段分支为橙色虚线，并写明“尚未开展”；
- 页面上不存在原模板项目名、Peach、论文/专利示例或红色提示文字。

- [ ] **Step 3: 如有视觉问题，只调整坐标或删减次要字段后重新生成**

允许的修改：

- 增加框间距；
- 缩短辅助字段；
- 删除 MoveIt2 证据带而保留 TB4；
- 将长标题从 32 pt 降至 30 pt；
- 将表格正文从 11.5 pt 降至 11 pt。

不允许的修改：

- 将 SVG 内部字段降到 15 pt 以下；
- 删除成果边界说明；
- 将 Issue、报警或候选改写为漏洞；
- 将 TB4 多样性数据改写为覆盖率；
- 修改源模板。

- [ ] **Step 4: 最终验证 Git 变更范围**

Run:

```bash
git status --short
git diff --check
git diff --stat -- 人才项目/PPT_第2-3部分
```

Expected: 本计划相关变更全部位于 `人才项目/PPT_第2-3部分/`；没有意外修改源模板或项目源码。

- [ ] **Step 5: 提交最终视觉修订**

只有确实产生修订时执行：

```bash
git add 人才项目/PPT_第2-3部分
git commit -m "fix: polish RoboFuzz PPT layout and rendering"
```

- [ ] **Step 6: 最终交付说明**

向用户提供以下可点击文件链接：

- 成品 PPTX；
- 七页预览图；
- 内容说明；
- 四个 SVG 图所在目录；
- 设计说明和本实施计划。

同时明确：源模板未修改；Linux/LibreOffice 已通过渲染检查；仍建议在 Windows PowerPoint 中进行一次微软雅黑字体和表格行高检查。
