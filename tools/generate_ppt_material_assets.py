#!/usr/bin/env python3
"""Generate editable SVG and high-resolution PNG assets for the PPT material pack."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle


BLUE = "#1769AA"
CYAN = "#00A6C8"
NAVY = "#16324F"
GREEN = "#2E9D63"
ORANGE = "#F39C3D"
RED = "#D95D5D"
GRAY = "#7A8793"
LIGHT_BLUE = "#EAF4FB"
LIGHT_CYAN = "#E8F8FA"
LIGHT_GREEN = "#EAF7F0"
LIGHT_ORANGE = "#FFF3E5"
LIGHT_RED = "#FCEDED"
TEXT = "#17324D"
GRID = "#D9E4EC"
WHITE = "#FFFFFF"

FONT_REGULAR = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
FONT_BOLD = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")


def configure_fonts() -> tuple[font_manager.FontProperties, font_manager.FontProperties]:
    if not FONT_REGULAR.exists() or not FONT_BOLD.exists():
        raise FileNotFoundError("Noto Sans CJK font files are required for Chinese charts")
    regular = font_manager.FontProperties(fname=str(FONT_REGULAR))
    bold = font_manager.FontProperties(fname=str(FONT_BOLD))
    mpl.rcParams.update(
        {
            "font.family": regular.get_name(),
            "axes.unicode_minus": False,
            "svg.fonttype": "none",
            "figure.facecolor": WHITE,
            "axes.facecolor": WHITE,
            "savefig.facecolor": WHITE,
        }
    )
    return regular, bold


REGULAR, BOLD = configure_fonts()


def load_chart_data(csv_path: Path) -> dict[str, list[dict[str, str]]]:
    grouped: dict[str, list[dict[str, str]]] = {}
    if not csv_path.exists():
        return grouped
    with csv_path.open(encoding="utf-8-sig", newline="") as fh:
        for row in csv.DictReader(fh):
            grouped.setdefault(row["chart"], []).append(row)
    return grouped


def save_pair(fig: plt.Figure, output_dir: Path, stem: str) -> None:
    fig.savefig(output_dir / f"{stem}.svg", bbox_inches="tight", pad_inches=0.15)
    fig.savefig(
        output_dir / f"{stem}.png",
        dpi=160,
        bbox_inches="tight",
        pad_inches=0.15,
    )
    plt.close(fig)


def title_block(fig: plt.Figure, title: str, subtitle: str) -> None:
    fig.text(0.04, 0.94, title, fontproperties=BOLD, fontsize=25, color=NAVY, va="top")
    fig.text(0.04, 0.89, subtitle, fontproperties=REGULAR, fontsize=12.5, color=GRAY, va="top")
    fig.add_artist(Rectangle((0.04, 0.862), 0.92, 0.006, transform=fig.transFigure, color=CYAN, lw=0))


def rounded_box(
    ax: plt.Axes,
    xy: tuple[float, float],
    width: float,
    height: float,
    label: str,
    facecolor: str,
    edgecolor: str = BLUE,
    fontsize: float = 12,
    text_color: str = TEXT,
) -> None:
    x, y = xy
    patch = FancyBboxPatch(
        (x, y),
        width,
        height,
        boxstyle="round,pad=0.012,rounding_size=0.02",
        facecolor=facecolor,
        edgecolor=edgecolor,
        linewidth=1.8,
    )
    ax.add_patch(patch)
    ax.text(
        x + width / 2,
        y + height / 2,
        label,
        ha="center",
        va="center",
        fontproperties=BOLD,
        fontsize=fontsize,
        color=text_color,
        linespacing=1.25,
    )


def arrow(ax: plt.Axes, start: tuple[float, float], end: tuple[float, float], color: str = BLUE) -> None:
    ax.add_patch(
        FancyArrowPatch(
            start,
            end,
            arrowstyle="-|>",
            mutation_scale=15,
            linewidth=1.8,
            color=color,
            shrinkA=3,
            shrinkB=3,
        )
    )


def method_pipeline(output_dir: Path) -> None:
    fig, ax = plt.subplots(figsize=(16, 9))
    fig.subplots_adjust(left=0.035, right=0.965, bottom=0.07, top=0.84)
    title_block(
        fig,
        "规范驱动语义模糊测试完整链路",
        "从多源系统规范到可执行 Oracle、反馈调度、仿真证据与上游沟通",
    )
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    labels = [
        ("多源规范\nsystem_doc", LIGHT_BLUE),
        ("格式感知预处理\n8 类 chunker", LIGHT_BLUE),
        ("SpecBlock\n来源与语义索引", LIGHT_CYAN),
        ("OracleIR\n结构化约束", LIGHT_CYAN),
        ("Agent 结合目标上下文\n生成/修订可执行 Oracle", LIGHT_GREEN),
        ("语义种子与变异\n反馈映射与 SeedQueue", LIGHT_GREEN),
        ("ROS 2 仿真实验\n五类现代目标", LIGHT_ORANGE),
        ("rosbag / ULG / 真值\n复算、回放与根因归并", LIGHT_ORANGE),
        ("公开 Issue 与成果转化\n按证据等级表达", LIGHT_RED),
    ]
    xs = [0.025, 0.222, 0.419, 0.616, 0.813]
    top_y = 0.60
    w, h = 0.162, 0.17
    for i in range(5):
        rounded_box(ax, (xs[i], top_y), w, h, labels[i][0], labels[i][1], fontsize=11.2)
    for i in range(4):
        arrow(
            ax,
            (xs[i] + w + 0.004, top_y + h / 2),
            (xs[i + 1] - 0.004, top_y + h / 2),
        )

    # Continue as a snake: the final top-row stage drops to the right-most
    # bottom box, then the evidence flow moves from right to left.  This keeps
    # the connector short and avoids a distracting full-width diagonal.
    bottom_xs = [0.82, 0.59, 0.36, 0.13]
    bottom_y = 0.22
    for j, idx in enumerate(range(5, 9)):
        rounded_box(ax, (bottom_xs[j] - 0.08, bottom_y), 0.18, h, labels[idx][0], labels[idx][1], fontsize=11.0)
    for j in range(3):
        arrow(
            ax,
            (bottom_xs[j] - 0.084, bottom_y + h / 2),
            (bottom_xs[j + 1] + 0.104, bottom_y + h / 2),
        )
    arrow(ax, (xs[4] + w / 2, top_y), (bottom_xs[0], bottom_y + h), color=CYAN)

    ax.text(0.05, 0.07, "已形成仿真侧闭环", fontproperties=BOLD, fontsize=12.5, color=GREEN)
    ax.text(
        0.20,
        0.07,
        "待补：完整传感器/消息事件扰动、严格覆盖率对照、Go2/ECO65-B 实物与虚实差分、CNVD/CNNVD",
        fontproperties=REGULAR,
        fontsize=11.5,
        color=RED,
    )
    save_pair(fig, output_dir, "method_pipeline")


def proposal_alignment(output_dir: Path) -> None:
    fig, ax = plt.subplots(figsize=(16, 9))
    fig.subplots_adjust(left=0.035, right=0.965, bottom=0.08, top=0.84)
    title_block(
        fig,
        "开题三条技术路线与当前工作的对应",
        "绿色为已形成较强证据，橙色为部分实现，灰红色为项目最终闭环仍待完成",
    )
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    rows = [
        (
            "路线一：定义正确行为",
            "LLM/规范 → DSL\n→ 行为 Oracle",
            "规范库、2749 SpecBlock\n151 份现代 OracleIR\n五类运行时 Oracle",
            "通用 IR 在线执行未统一\n复杂约束依赖 Agent +\n人工上下文化",
            GREEN,
        ),
        (
            "路线二：激活漏洞状态",
            "四维输入 + 路径结构\n+ 语义反馈",
            "8 类语义策略\n约 118 个声明反馈维度\n有界 SeedQueue、行为序列",
            "传感器专用注入和\n消息乱序/丢包较弱\n显式 IPDG 尚未实现",
            ORANGE,
        ),
        (
            "路线三：识别行为偏离",
            "话题插桩 + 场景回溯\n+ 虚实差分",
            "rosbag / ULG / 独立真值\n离线复算、回放\n与根因归并",
            "完成仿真侧\nGo2/ECO65-B 实物及\nL_sim/L_real 对齐未开展",
            ORANGE,
        ),
    ]
    y_values = [0.65, 0.40, 0.15]
    for (route, planned, done, gap, status), y in zip(rows, y_values):
        rounded_box(ax, (0.025, y), 0.18, 0.15, route, LIGHT_BLUE, fontsize=12)
        arrow(ax, (0.205, y + 0.075), (0.235, y + 0.075), color=GRAY)
        rounded_box(ax, (0.235, y), 0.205, 0.15, planned, WHITE, edgecolor=GRAY, fontsize=10.7)
        arrow(ax, (0.44, y + 0.075), (0.47, y + 0.075), color=GRAY)
        rounded_box(
            ax,
            (0.47, y),
            0.255,
            0.15,
            done,
            LIGHT_GREEN if status == GREEN else LIGHT_ORANGE,
            edgecolor=status,
            fontsize=9.9,
        )
        arrow(ax, (0.725, y + 0.075), (0.755, y + 0.075), color=GRAY)
        rounded_box(ax, (0.755, y), 0.22, 0.15, gap, LIGHT_RED, edgecolor=RED, fontsize=9.5)

    ax.text(0.115, 0.86, "项目问题", fontproperties=BOLD, fontsize=12.5, color=NAVY, ha="center")
    ax.text(0.338, 0.86, "原方案", fontproperties=BOLD, fontsize=12.5, color=NAVY, ha="center")
    ax.text(0.598, 0.86, "本人已形成的工作", fontproperties=BOLD, fontsize=12.5, color=GREEN, ha="center")
    ax.text(0.865, 0.86, "待完成/边界", fontproperties=BOLD, fontsize=12.5, color=RED, ha="center")
    save_pair(fig, output_dir, "proposal_alignment")


def specblock_oracleir_counts(output_dir: Path, grouped: dict[str, list[dict[str, str]]]) -> None:
    defaults = {
        "MoveIt2 Panda": (632, 27),
        "TurtleBot4": (598, 18),
        "PX4 v1.17": (561, 26),
        "SLAM Toolbox": (229, 38),
        "Nav2": (729, 42),
    }
    rows = grouped.get("specblocks_oracleir") or grouped.get("specblock_oracleir_counts") or []
    parsed: dict[str, dict[str, float]] = {}
    for row in rows:
        parsed.setdefault(row["category"], {})[row["series"]] = float(row["value"])
    if parsed:
        data = {
            category: (
                int(series.get("SpecBlock", series.get("specblocks", 0))),
                int(series.get("OracleIR", series.get("oracleir", 0))),
            )
            for category, series in parsed.items()
        }
        if any(a == 0 or b == 0 for a, b in data.values()):
            data = defaults
    else:
        data = defaults

    names = list(data)
    blocks = [data[n][0] for n in names]
    irs = [data[n][1] for n in names]
    fig, axes = plt.subplots(1, 2, figsize=(16, 9), gridspec_kw={"width_ratios": [1.25, 1]})
    fig.subplots_adjust(left=0.08, right=0.96, bottom=0.16, top=0.79, wspace=0.26)
    title_block(
        fig,
        "五类现代目标的规范处理与 OracleIR 规模",
        "当前工作区：425 个输入文件 → 2749 个 SpecBlock → 151 份现代 OracleIR；数量不等于全部已在线执行",
    )
    colors = [BLUE, CYAN, "#3B82C4", "#39A59B", "#5579B8"]
    x = range(len(names))
    axes[0].bar(x, blocks, color=colors, width=0.64)
    axes[0].set_title("SpecBlock 数量", fontproperties=BOLD, fontsize=17, color=NAVY, pad=14)
    axes[0].set_xticks(list(x), names, rotation=18, ha="right", fontproperties=REGULAR, fontsize=11)
    axes[0].set_ylim(0, max(blocks) * 1.18)
    axes[0].grid(axis="y", color=GRID, linewidth=0.8)
    axes[0].set_axisbelow(True)
    for i, value in enumerate(blocks):
        axes[0].text(i, value + 14, str(value), ha="center", fontproperties=BOLD, fontsize=12, color=TEXT)

    axes[1].bar(x, irs, color=colors, width=0.64)
    axes[1].set_title("OracleIR 数量", fontproperties=BOLD, fontsize=17, color=NAVY, pad=14)
    axes[1].set_xticks(list(x), names, rotation=18, ha="right", fontproperties=REGULAR, fontsize=11)
    axes[1].set_ylim(0, max(irs) * 1.22)
    axes[1].grid(axis="y", color=GRID, linewidth=0.8)
    axes[1].set_axisbelow(True)
    for i, value in enumerate(irs):
        axes[1].text(i, value + 0.8, str(value), ha="center", fontproperties=BOLD, fontsize=12, color=TEXT)

    fig.text(
        0.5,
        0.055,
        "另保留 legacy PX4 OracleIR 35 份；现代生成规格多数为本地工作区资产，不能表述为全部已提交或全部接入运行时。",
        ha="center",
        fontproperties=REGULAR,
        fontsize=11.5,
        color=GRAY,
    )
    save_pair(fig, output_dir, "specblock_oracleir_counts")


def experiment_root_cause_funnel(output_dir: Path) -> None:
    fig, ax = plt.subplots(figsize=(16, 9))
    fig.subplots_adjust(left=0.035, right=0.965, bottom=0.09, top=0.84)
    title_block(
        fig,
        "从高频报警收敛到少量可解释候选根因",
        "错误轮次/帧是检测信号，不是漏洞数量；通过错误签名、共现、级联、独立真值和离线复算进行归并",
    )
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    rows = [
        ("TurtleBot4", "928 个 error rounds", "842 个唯一输入\n781 个错误输入签名", "1 条主要加速度/命令路径链", "当前归因仍需上游完整 launch 对照"),
        ("MoveIt2", "224 个 error frames", "166 绝对加速度命中\n219 scaling 命中；97% 共现", "1 个 TOPP-RA 根因族\n2 个契约投影", "不是 224 或 385 个漏洞"),
        ("PX4 v1.17", "12 份 error / ULG 候选", "4 个持续 tilt+rate\n高信号样本", "1 条主控制响应链\n两档严重度", "尚无最小复现和 v1.17 上游确认"),
        ("SLAM", "18 个语义报警轮次", "19 个标签（1 轮双标签）\n7 strong / 5 moderate\n7 borderline", "2 个强/中候选机制\n+1 个边界族", "性能偏轨候选，不等同安全漏洞"),
    ]
    y_positions = [0.68, 0.49, 0.30, 0.11]
    for (name, raw, evidence, root, note), y in zip(rows, y_positions):
        rounded_box(ax, (0.02, y), 0.13, 0.12, name, LIGHT_BLUE, fontsize=12.5)
        rounded_box(ax, (0.18, y), 0.19, 0.12, raw, LIGHT_ORANGE, edgecolor=ORANGE, fontsize=10.8)
        arrow(ax, (0.37, y + 0.06), (0.405, y + 0.06), color=GRAY)
        rounded_box(ax, (0.405, y), 0.22, 0.12, evidence, WHITE, edgecolor=GRAY, fontsize=9.8)
        arrow(ax, (0.625, y + 0.06), (0.66, y + 0.06), color=GRAY)
        rounded_box(ax, (0.66, y), 0.18, 0.12, root, LIGHT_GREEN, edgecolor=GREEN, fontsize=10.2)
        rounded_box(ax, (0.86, y), 0.12, 0.12, note, LIGHT_RED, edgecolor=RED, fontsize=8.6)

    ax.text(0.275, 0.86, "原始实验信号", fontproperties=BOLD, fontsize=12.5, color=ORANGE, ha="center")
    ax.text(0.515, 0.86, "复算/共现/去重证据", fontproperties=BOLD, fontsize=12.5, color=NAVY, ha="center")
    ax.text(0.75, 0.86, "候选根因级解释", fontproperties=BOLD, fontsize=12.5, color=GREEN, ha="center")
    ax.text(0.92, 0.86, "边界", fontproperties=BOLD, fontsize=12.5, color=RED, ha="center")
    save_pair(fig, output_dir, "experiment_root_cause_funnel")


def tb4_diversity_comparison(
    output_dir: Path,
    grouped: dict[str, list[dict[str, str]]],
) -> None:
    defaults = {
        "p0p1": {"Unique inputs": 629, "Dominant signature repeats": 156, "Error rounds": 941},
        "semantic3": {"Unique inputs": 842, "Dominant signature repeats": 21, "Error rounds": 928},
    }
    parsed: dict[str, dict[str, float]] = {}
    for row in grouped.get("tb4_diversity", []):
        parsed.setdefault(row["category"], {})[row["series"]] = float(row["value"])
    data = parsed if all(name in parsed for name in defaults) else defaults

    versions = ["p0p1", "semantic3"]
    unique = [data[v].get("Unique inputs", defaults[v]["Unique inputs"]) for v in versions]
    errors = [data[v].get("Error rounds", defaults[v]["Error rounds"]) for v in versions]
    repeats = [
        data[v].get("Dominant signature repeats", defaults[v]["Dominant signature repeats"])
        for v in versions
    ]

    fig, axes = plt.subplots(1, 2, figsize=(16, 9), gridspec_kw={"width_ratios": [1.35, 1]})
    fig.subplots_adjust(left=0.075, right=0.96, bottom=0.15, top=0.79, wspace=0.26)
    title_block(
        fig,
        "TurtleBot4 semantic3 组合优化：多样性提升，重复显著下降",
        "两轮均为 1000 轮；组合包含 feedback reset、重复签名重入队抑制与 reversal 变体轮换，不是单变量消融",
    )

    x = [0, 1]
    width = 0.34
    axes[0].bar([v - width / 2 for v in x], unique, width=width, color=BLUE, label="唯一输入")
    axes[0].bar([v + width / 2 for v in x], errors, width=width, color=ORANGE, label="错误轮次")
    axes[0].set_xticks(x, ["p0p1", "semantic3"], fontproperties=REGULAR, fontsize=12)
    axes[0].set_ylim(0, 1050)
    axes[0].set_title("探索规模与报警总量", fontproperties=BOLD, fontsize=17, color=NAVY, pad=14)
    axes[0].grid(axis="y", color=GRID, linewidth=0.8)
    axes[0].set_axisbelow(True)
    for xpos, value in zip([v - width / 2 for v in x], unique):
        axes[0].text(xpos, value + 20, f"{int(value)}", ha="center", fontproperties=BOLD, fontsize=12, color=TEXT)
    for xpos, value in zip([v + width / 2 for v in x], errors):
        axes[0].text(xpos, value + 20, f"{int(value)}", ha="center", fontproperties=BOLD, fontsize=12, color=TEXT)
    legend = axes[0].legend(loc="upper left", frameon=False, prop=REGULAR)
    for item in legend.get_texts():
        item.set_color(TEXT)
    unique_gain = (unique[1] - unique[0]) / unique[0] * 100
    axes[0].text(
        0.5,
        0.10,
        f"唯一输入 +{unique_gain:.1f}%",
        transform=axes[0].transAxes,
        ha="center",
        fontproperties=BOLD,
        fontsize=14,
        color=GREEN,
    )

    axes[1].bar(x, repeats, width=0.58, color=[GRAY, GREEN])
    axes[1].set_xticks(x, ["p0p1", "semantic3"], fontproperties=REGULAR, fontsize=12)
    axes[1].set_ylim(0, 180)
    axes[1].set_title("最高重复签名次数", fontproperties=BOLD, fontsize=17, color=NAVY, pad=14)
    axes[1].grid(axis="y", color=GRID, linewidth=0.8)
    axes[1].set_axisbelow(True)
    for xpos, value in zip(x, repeats):
        axes[1].text(xpos, value + 5, f"{int(value)}", ha="center", fontproperties=BOLD, fontsize=14, color=TEXT)
    repeat_drop = (repeats[0] - repeats[1]) / repeats[0] * 100
    axes[1].text(
        0.5,
        0.72,
        f"下降 {repeat_drop:.1f}%",
        transform=axes[1].transAxes,
        ha="center",
        fontproperties=BOLD,
        fontsize=16,
        color=GREEN,
    )

    fig.text(
        0.5,
        0.055,
        "错误轮次 941 → 928，基本保持；该图证明输入多样性和重复治理改善，不等同于控制语义约束覆盖率已提升 20%。",
        ha="center",
        fontproperties=REGULAR,
        fontsize=11.5,
        color=GRAY,
    )
    save_pair(fig, output_dir, "tb4_diversity_comparison")


def kpi_status(output_dir: Path) -> None:
    labels = [
        "1  控制语义约束不少于 50 条",
        "2  指令/传感器/控制参数/消息事件四类输入",
        "3  语义相关变异算子不少于 5 类",
        "4  控制语义偏轨判定与场景回溯验证",
        "5  相对 RoboFuzz 语义约束覆盖率提升 ≥20%",
        "6  ≥2 类典型平台、≥5 个未公开漏洞并获 CNVD/CNNVD",
    ]
    state_labels = ["达到", "部分达到", "达到", "仿真侧达到", "待正式证据", "待正式证据"]
    statuses = [
        "规模达到；运行映射清单待固化",
        "指令强；参数中等；事件部分支持；传感器偏弱",
        "8 类跨目标语义策略",
        "仿真侧达到；实物回放未完成",
        "有提升迹象；缺同预算严格对照",
        "有多平台候选与 Issue；无实物/编号闭环",
    ]
    colors = [GREEN, ORANGE, GREEN, ORANGE, RED, RED]
    fills = [LIGHT_GREEN, LIGHT_ORANGE, LIGHT_GREEN, LIGHT_ORANGE, LIGHT_RED, LIGHT_RED]
    fig, ax = plt.subplots(figsize=(16, 9))
    fig.subplots_adjust(left=0.035, right=0.965, bottom=0.09, top=0.84)
    title_block(
        fig,
        "书面开题报告六项指标的阶段状态",
        "采用离散状态而非进度百分比：达到、仿真侧达到、部分达到、待形成正式证据",
    )
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.text(0.17, 0.86, "正式指标", ha="center", fontproperties=BOLD, fontsize=12.5, color=NAVY)
    ax.text(0.415, 0.86, "阶段状态", ha="center", fontproperties=BOLD, fontsize=12.5, color=NAVY)
    ax.text(0.72, 0.86, "当前证据与边界", ha="center", fontproperties=BOLD, fontsize=12.5, color=NAVY)
    y_values = [0.71, 0.59, 0.47, 0.35, 0.23, 0.11]
    for label, state, status, color, fill, y in zip(
        labels, state_labels, statuses, colors, fills, y_values
    ):
        rounded_box(ax, (0.015, y), 0.31, 0.09, label, WHITE, edgecolor=GRAY, fontsize=9.6)
        rounded_box(ax, (0.345, y), 0.14, 0.09, state, fill, edgecolor=color, fontsize=10.8, text_color=color)
        rounded_box(ax, (0.505, y), 0.47, 0.09, status, fill, edgecolor=color, fontsize=10.1)
    fig.text(
        0.5,
        0.04,
        "正式验收不能写成“六项全部完成”：第 1、3 项有直接数量证据；第 4 项完成仿真侧；第 2、5、6 项仍需补齐。",
        ha="center",
        fontproperties=BOLD,
        fontsize=11.5,
        color=NAVY,
    )
    save_pair(fig, output_dir, "kpi_status")


def issues_by_platform(output_dir: Path) -> None:
    platforms = ["TurtleBot3", "PX4", "MoveIt/MoveIt2", "TB4/Create3", "slam_toolbox"]
    open_counts = [6, 1, 6, 9, 0]
    closed_counts = [2, 4, 0, 0, 6]
    x = range(len(platforms))
    fig, ax = plt.subplots(figsize=(16, 9))
    fig.subplots_adjust(left=0.08, right=0.96, bottom=0.18, top=0.80)
    title_block(
        fig,
        "各平台公开 Issue 的开放与关闭状态",
        "2026-09-17 GitHub 快照：34 条 Issue = 22 Open + 12 Closed；Issue 数量不等于独立漏洞或维护者确认数",
    )
    ax.bar(x, open_counts, label="Open", color=GREEN, width=0.62)
    ax.bar(x, closed_counts, bottom=open_counts, label="Closed", color=GRAY, width=0.62)
    ax.set_xticks(list(x), platforms, fontproperties=REGULAR, fontsize=12)
    ax.set_ylim(0, 11)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for i, (opened, closed) in enumerate(zip(open_counts, closed_counts)):
        if opened:
            ax.text(i, opened / 2, str(opened), ha="center", va="center", color=WHITE, fontproperties=BOLD, fontsize=13)
        if closed:
            ax.text(i, opened + closed / 2, str(closed), ha="center", va="center", color=WHITE, fontproperties=BOLD, fontsize=13)
        ax.text(i, opened + closed + 0.25, str(opened + closed), ha="center", color=TEXT, fontproperties=BOLD, fontsize=13)
    legend = ax.legend(loc="upper right", frameon=False, prop=REGULAR)
    for text in legend.get_texts():
        text.set_color(TEXT)
    ax.set_ylabel("Issue 数量", fontproperties=REGULAR, fontsize=12, color=TEXT)
    save_pair(fig, output_dir, "issues_by_platform_status")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--data", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    grouped = load_chart_data(args.data)
    method_pipeline(args.output_dir)
    proposal_alignment(args.output_dir)
    specblock_oracleir_counts(args.output_dir, grouped)
    experiment_root_cause_funnel(args.output_dir)
    tb4_diversity_comparison(args.output_dir, grouped)
    kpi_status(args.output_dir)
    issues_by_platform(args.output_dir)


if __name__ == "__main__":
    main()
