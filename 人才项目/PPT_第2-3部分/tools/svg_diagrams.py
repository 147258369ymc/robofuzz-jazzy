from __future__ import annotations

from dataclasses import dataclass, field
from html import escape
from typing import Iterable


W = 2400
H = 800
FONT = "Microsoft YaHei, Noto Sans CJK SC, sans-serif"
COLORS = {
    "navy": "#002060", "blue": "#0070C0", "line": "#6592F5",
    "cyan": "#DDF4FF", "orange": "#EF962D", "orange_fill": "#FFF3E2",
    "green": "#289D64", "green_fill": "#EAF7EF", "red": "#C00000",
    "gray": "#F3F6F8", "gray_text": "#5F6B76", "black": "#111111",
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

    def rect(self, x: float, y: float, width: float, height: float, *, fill: str = COLORS["white"],
             stroke: str = COLORS["line"], stroke_width: float = 3, dash: str | None = None) -> None:
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        self.items.append(
            f'<rect x="{x}" y="{y}" width="{width}" height="{height}" fill="{fill}" '
            f'stroke="{stroke}" stroke-width="{stroke_width}"{dash_attr}/>'
        )

    def text(self, x: float, y: float, value: str, *, size: float = 42, fill: str = COLORS["black"],
             weight: str | int = "400", anchor: str = "start") -> None:
        if size < 42:
            raise ValueError("All SVG text must be at least 42 units")
        self.items.append(
            f'<text x="{x}" y="{y}" font-family="{FONT}" font-size="{size}" fill="{fill}" '
            f'font-weight="{weight}" text-anchor="{anchor}">{escape(value)}</text>'
        )

    def multiline(self, x: float, y: float, values: Iterable[str], *, size: float = 42,
                  leading: float = 50, fill: str = COLORS["black"], weight: str | int = "400",
                  anchor: str = "start") -> None:
        for index, value in enumerate(values):
            self.text(x, y + index * leading, value, size=size, fill=fill, weight=weight, anchor=anchor)

    def polyline(self, points: Iterable[tuple[float, float]], *, color: str = COLORS["blue"],
                 width: float = 5, arrow: str | None = "blue", dash: str | None = None) -> None:
        pair_list = list(points)
        if len(pair_list) < 2:
            raise ValueError("Polyline needs at least two points")
        for first, second in zip(pair_list, pair_list[1:]):
            if first[0] != second[0] and first[1] != second[1]:
                raise ValueError("Polyline segments must be orthogonal")
        marker = f' marker-end="url(#arrow-{arrow})"' if arrow else ""
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        rendered = " ".join(f"{x},{y}" for x, y in pair_list)
        self.items.append(
            f'<polyline points="{rendered}" fill="none" stroke="{color}" stroke-width="{width}" '
            f'stroke-linejoin="miter"{marker}{dash_attr}/>'
        )

    def render(self) -> str:
        defs = "".join(
            f'<marker id="arrow-{name}" viewBox="0 0 12 12" refX="11" refY="6" markerWidth="10" '
            f'markerHeight="10" orient="auto"><path d="M 0 0 L 12 6 L 0 12 z" fill="{color}"/></marker>'
            for name, color in (("blue", COLORS["blue"]), ("green", COLORS["green"]),
                                ("orange", COLORS["orange"]), ("gray", COLORS["gray_text"]),
                                ("black", COLORS["black"]))
        )
        return (
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">'
            f'<defs>{defs}</defs><rect x="0" y="0" width="{W}" height="{H}" fill="{COLORS["white"]}"/>'
            f'{"".join(self.items)}</svg>\n'
        )


def label(canvas: SvgCanvas, x: float, y: float, value: str, *, size: float = 42,
          fill: str = COLORS["navy"], anchor: str = "start") -> None:
    canvas.text(x, y, value, size=size, fill=fill, weight="700", anchor=anchor)


def box(canvas: SvgCanvas, x: float, y: float, width: float, height: float, title: str,
        lines: Iterable[str] = (), *, fill: str = COLORS["white"], stroke: str = COLORS["line"],
        title_fill: str = COLORS["navy"], title_size: float = 48, line_size: float = 42,
        leading: float = 50, content_offset: float = 112, dash: str | None = None) -> None:
    canvas.rect(x, y, width, height, fill=fill, stroke=stroke, dash=dash)
    label(canvas, x + 22, y + 55, title, size=title_size, fill=title_fill)
    canvas.multiline(x + 22, y + content_offset, lines, size=line_size, leading=leading)


def draw_overview() -> str:
    c = SvgCanvas()
    # fixed regions: inputs, semantic loop, and reusable assets
    c.rect(20, 80, 330, 600, fill=COLORS["gray"], stroke=COLORS["navy"], stroke_width=4)
    label(c, 42, 140, "输入", size=58)
    box(c, 42, 150, 286, 150, "系统规范", ("文档·配置", "接口·源码"), fill=COLORS["white"], leading=42, content_offset=95)
    box(c, 42, 322, 286, 150, "目标系统", ("话题·参数", "状态机"), fill=COLORS["cyan"], leading=42, content_offset=95)
    box(c, 42, 494, 286, 150, "实验场景", ("平台·任务", "仿真环境"), fill=COLORS["white"], leading=42, content_offset=95)

    c.rect(390, 40, 1480, 680, fill=COLORS["white"], stroke=COLORS["navy"], stroke_width=4)
    label(c, 420, 105, "规范驱动的语义模糊测试闭环", size=58)
    box(c, 430, 150, 390, 360, "任务一",
        ("定义正确行为", "规范分块", "结构化约束", "运行时Oracle"), fill=COLORS["cyan"], title_size=48)
    box(c, 935, 150, 390, 360, "任务二",
        ("激活异常路径", "行为种子", "语义变异", "反馈调度"), fill=COLORS["orange_fill"], stroke=COLORS["orange"])
    box(c, 1440, 150, 390, 360, "任务三",
        ("验证行为偏轨", "在线判定", "复算回放", "根因归并"), fill=COLORS["green_fill"], stroke=COLORS["green"])
    c.rect(430, 588, 1400, 92, fill=COLORS["gray"], stroke=COLORS["gray_text"])
    label(c, 455, 628, "PX4　TurtleBot4　MoveIt2", size=42, fill=COLORS["navy"])
    # Keep the platform summary to a single, comfortably contained line.
    c.text(455, 670, "SLAM　Nav2｜五类目标｜成熟度不一", size=42, fill=COLORS["gray_text"])

    c.rect(1920, 80, 450, 600, fill=COLORS["gray"], stroke=COLORS["navy"], stroke_width=4)
    label(c, 1945, 140, "可复用资产", size=58)
    box(c, 1945, 175, 400, 90, "可追溯规则", (), fill=COLORS["cyan"])
    box(c, 1945, 292, 400, 90, "高价值种子", (), fill=COLORS["orange_fill"], stroke=COLORS["orange"])
    box(c, 1945, 409, 400, 90, "候选证据包", (), fill=COLORS["green_fill"], stroke=COLORS["green"])
    box(c, 1945, 526, 400, 90, "公开沟通材料", (), fill=COLORS["white"])

    # Three input cards first merge on a dedicated trunk, then enter task one.
    for source_y in (225, 397, 569):
        c.polyline(((328, source_y), (370, source_y)), color=COLORS["blue"], arrow=None)
    c.polyline(((370, 225), (370, 569)), color=COLORS["blue"], arrow=None)
    c.polyline(((370, 397), (430, 397)), color=COLORS["blue"], arrow="blue")
    c.polyline(((820, 330), (935, 330)), color=COLORS["blue"], arrow="blue")
    c.polyline(((1325, 330), (1440, 330)), color=COLORS["blue"], arrow="blue")
    # Each task has its own correctly typed asset output.
    c.polyline(((820, 450), (820, 575), (1910, 575), (1910, 220), (1945, 220)), color=COLORS["blue"], arrow="blue")
    c.polyline(((1325, 450), (1325, 565), (1900, 565), (1900, 337), (1945, 337)), color=COLORS["blue"], arrow="blue")
    c.polyline(((1830, 330), (1885, 330), (1885, 454), (1945, 454)), color=COLORS["blue"], arrow="blue")
    # feedback is deliberately above the platform strip.
    c.polyline(((1635, 510), (1635, 550), (1130, 550), (1130, 510)), color=COLORS["green"], arrow="green", dash="12 10")
    c.polyline(((1635, 550), (625, 550), (625, 510)), color=COLORS["green"], arrow="green", dash="12 10")
    c.text(800, 540, "补充规则 · 调整输入", size=42, fill=COLORS["green"], weight="700")
    return c.render()


def draw_oracle_pipeline() -> str:
    c = SvgCanvas()
    c.text(1200, 36, "当前工作区 · 五个现代目标", size=42, fill=COLORS["gray_text"], anchor="middle")
    c.rect(20, 50, 300, 550, fill=COLORS["gray"], stroke=COLORS["navy"], stroke_width=4)
    label(c, 42, 112, "阶段1", size=56)
    c.multiline(42, 175, ("多源规范", "文档/注释", "配置/Launch", "ROS接口", "源码/模型"), leading=53)
    c.rect(42, 448, 256, 94, fill=COLORS["cyan"], stroke=COLORS["blue"])
    c.multiline(62, 488, ("425个", "输入文件"), size=42, leading=42, fill=COLORS["blue"], weight="700")

    c.rect(350, 50, 330, 550, fill=COLORS["white"], stroke=COLORS["navy"], stroke_width=4)
    label(c, 372, 112, "阶段2", size=56)
    c.multiline(372, 175, ("预处理", "格式识别", "分块/归一", "保留来源"), leading=53)
    c.rect(372, 350, 286, 190, fill=COLORS["cyan"], stroke=COLORS["blue"])
    label(c, 392, 408, "SpecBlock", size=48, fill=COLORS["blue"])
    c.multiline(392, 450, ("2749个", "内容/路径/", "上下文"), leading=42)

    c.rect(710, 50, 370, 550, fill=COLORS["white"], stroke=COLORS["navy"], stroke_width=4)
    label(c, 732, 112, "阶段3", size=56)
    c.multiline(
        732,
        170,
        ("Agent生成", "检索相关规范", "生成候选IR", "结构校验", "语义规则检查"),
        leading=42,
    )
    c.rect(732, 414, 326, 127, fill=COLORS["cyan"], stroke=COLORS["blue"])
    label(c, 752, 472, "OracleIR", size=48, fill=COLORS["blue"])
    c.text(752, 522, "151份现代IR", size=42)
    c.text(732, 397, "失败则修订", size=42, fill=COLORS["orange"], weight="700")
    c.polyline(
        ((1035, 397), (1060, 397), (1060, 360), (1015, 360)),
        color=COLORS["orange"],
        arrow="orange",
        dash="10 8",
    )

    c.rect(1130, 50, 570, 550, fill=COLORS["white"], stroke=COLORS["navy"], stroke_width=4)
    c.rect(1142, 72, 546, 506, fill=COLORS["orange_fill"], stroke=COLORS["orange"], stroke_width=4, dash="14 10")
    label(c, 1165, 130, "自动化边界", size=48, fill=COLORS["orange"])
    label(c, 1165, 190, "阶段4", size=56)
    c.multiline(1165, 238, ("目标上下文化", "话题/字段　状态/时间窗", "容差/噪声　质量门控", "运行时判断"), leading=44)
    c.rect(1165, 385, 500, 72, fill=COLORS["white"], stroke=COLORS["orange"])
    c.multiline(1185, 414, ("候选规则｜目标适配", "并非全部IR上线"), size=42, leading=38, fill=COLORS["orange"], weight="700")
    c.rect(1165, 470, 500, 95, fill=COLORS["green_fill"], stroke=COLORS["green"])
    c.multiline(1190, 510, ("正常　/　偏轨", "不可判定"), size=42, leading=44, fill=COLORS["green"], weight="700")

    c.rect(1750, 50, 630, 550, fill=COLORS["gray"], stroke=COLORS["navy"], stroke_width=4)
    label(c, 1772, 112, "OracleIR", size=56)
    c.multiline(1772, 170, ("关键字段", "看什么／观察量", "阈值从哪来／参数·单位", "怎么算／派生量", "何时算错／断言", "何时生效／作用域", "看多久／时间窗"), leading=48)
    c.rect(1772, 510, 586, 55, fill=COLORS["white"], stroke=COLORS["gray_text"])
    c.text(1794, 550, "来源证据：SpecBlock ID", size=42, fill=COLORS["gray_text"])

    c.rect(20, 650, 2360, 120, fill=COLORS["gray"], stroke=COLORS["gray_text"], stroke_width=3)
    label(c, 45, 708, "反向追溯", size=48)
    c.text(360, 708, "原始规范位置", size=42)
    c.text(760, 708, "SpecBlock", size=42)
    c.text(1125, 708, "OracleIR", size=42)
    c.text(1450, 708, "运行时规则", size=42)
    c.text(1850, 708, "判定证据", size=42)
    c.polyline(((730, 730), (580, 730)), color=COLORS["gray_text"], arrow="gray")
    c.polyline(((1090, 730), (930, 730)), color=COLORS["gray_text"], arrow="gray")
    c.polyline(((1420, 730), (1260, 730)), color=COLORS["gray_text"], arrow="gray")
    c.polyline(((1815, 730), (1610, 730)), color=COLORS["gray_text"], arrow="gray")
    c.polyline(((320, 325), (350, 325)), color=COLORS["blue"], arrow="blue")
    c.polyline(((680, 325), (710, 325)), color=COLORS["blue"], arrow="blue")
    c.polyline(((1080, 325), (1130, 325), (1130, 421), (1165, 421)), color=COLORS["blue"], arrow="blue")
    c.polyline(((1700, 325), (1750, 325)), color=COLORS["blue"], arrow="blue")
    c.polyline(((2340, 170), (2340, 475), (2320, 475), (2320, 510)), color=COLORS["gray_text"], arrow="gray")
    return c.render()


def draw_seed_feedback() -> str:
    c = SvgCanvas()
    c.rect(20, 60, 300, 620, fill=COLORS["gray"], stroke=COLORS["navy"], stroke_width=4)
    label(c, 42, 120, "多维输入", size=56)
    c.multiline(42, 180, ("用户指令", "控制参数", "消息事件", "传感数据*"), leading=58)
    label(c, 42, 445, "时间属性", size=48)
    c.multiline(42, 500, ("t0 → t1 → …", "→ tn"), size=42, leading=42, fill=COLORS["blue"], weight="700")
    c.multiline(42, 570, ("顺序·持续/频率", "*专用扰动", "仍待补强"), leading=42, fill=COLORS["gray_text"])

    c.rect(360, 40, 1460, 680, fill=COLORS["white"], stroke=COLORS["navy"], stroke_width=4)
    box(c, 395, 62, 390, 293, "A",
        ("种子治理与选择", "容量 ≤ 50", "单种子 ≤ 5次", "近似去重/陈旧淘汰", "初始轮转/新鲜度"), fill=COLORS["cyan"], title_size=48, leading=42)
    box(c, 815, 85, 330, 270, "B",
        ("行为序列构造", "消息序列", "持续时间", "顺序/模式"), fill=COLORS["white"], title_size=48, leading=48)
    box(c, 1175, 70, 610, 285, "C｜反馈语义变异·映射",
        ("基础权重 + 反馈提升", "多轴　翻转　持续　边界", "反转　弧线　极值　回环"), fill=COLORS["orange_fill"], stroke=COLORS["orange"], title_size=48, leading=45)
    c.rect(1210, 287, 540, 58, fill=COLORS["white"], stroke=COLORS["orange"])
    c.text(1230, 328, "策略：", size=42, fill=COLORS["orange"], weight="700")
    c.text(1350, 328, "边界压力 → 边界推进", size=42, fill=COLORS["orange"], weight="700")

    c.rect(395, 410, 1390, 200, fill=COLORS["gray"], stroke=COLORS["gray_text"])
    label(c, 420, 465, "连续反馈带", size=48)
    label(c, 420, 515, "质量门控", size=42)
    c.text(420, 558, "就绪·可判定", size=42)
    label(c, 750, 515, "语义反馈", size=42)
    c.text(750, 558, "运动·任务", size=42)
    label(c, 1080, 515, "新颖性", size=42)
    c.text(1080, 558, "连续/错误签名", size=42)
    label(c, 1395, 515, "队列治理", size=42)
    c.text(1395, 558, "保留/抑制重复", size=42)
    c.text(420, 590, "证据不足 / 基础设施异常 → 不入队", size=42, fill=COLORS["gray_text"])

    c.rect(1860, 60, 520, 660, fill=COLORS["gray"], stroke=COLORS["navy"], stroke_width=4)
    label(c, 1885, 120, "目标执行与观测", size=56)
    c.multiline(1885, 165, ("TB4·PX4·MoveIt2", "SLAM·Nav2", "控制状态·任务结果", "运行质量"), leading=43)
    c.text(1885, 333, "按目标适配，成熟度不同", size=42, fill=COLORS["gray_text"])
    c.rect(1885, 350, 470, 360, fill=COLORS["white"], stroke=COLORS["blue"], stroke_width=4)
    label(c, 1908, 395, "TB4｜组合优化结果", size=42, fill=COLORS["blue"])
    c.multiline(1908, 435, ("唯一输入签名", "629→842｜+33.9%", "最高单一签名重复", "156→21｜下降86.5%"), size=42, leading=42)
    c.multiline(1908, 590, ("组合优化结果", "不等同于语义约束", "覆盖率"), size=42, leading=42, fill=COLORS["red"], weight="700")

    c.polyline(((320, 270), (395, 270)), color=COLORS["blue"], arrow="blue")
    c.polyline(((785, 220), (815, 220)), color=COLORS["blue"], arrow="blue")
    c.polyline(((1145, 220), (1175, 220)), color=COLORS["blue"], arrow="blue")
    c.polyline(((1785, 220), (1860, 220)), color=COLORS["blue"], arrow="blue")
    c.polyline(((1530, 355), (1530, 410)), color=COLORS["blue"], arrow="blue")
    c.polyline(((680, 410), (680, 380), (590, 380), (590, 355)), color=COLORS["green"], arrow="green", dash="12 10")
    return c.render()


def draw_evidence_root_cause() -> str:
    c = SvgCanvas()
    c.rect(20, 30, 1660, 610, fill=COLORS["white"], stroke=COLORS["navy"], stroke_width=4)
    # Four stages are large frames; their items remain text lines, avoiding a dense grid of tiny boxes.
    stage_widths = (340, 340, 330, 545)
    stage_xs = (45, 405, 765, 1115)
    stages = (
        ("①证据恢复", ("触发输入与参数", "Topic/Action/TF", "任务模式与状态", "rosbag/ULG", "仿真真值/", "任务结果", "按目标选用")),
        ("②质量门控", ("关键字段齐全", "时钟/样本对齐", "有效运动/", "有效任务", "基础设施异常", "可判定→Oracle", "不可判定≠报警")),
        ("③在线Oracle", ("状态前置", "窗口与持续性", "范围/时序约束", "跨话题一致性", "任务结果一致性", "偏轨标签", "报警帧/错误轮次")),
        ("④复核与根因归并", ("离线复算", "独立回放", "恢复同一触发输入", "交叉证据验证", "成立/边界/证伪", "同输入·同时段·同控制链", "错误签名去重→级联合并", "候选根因族")),
    )
    for x, width, (title, lines) in zip(stage_xs, stage_widths, stages):
        c.rect(x, 95, width, 500, fill=COLORS["gray"], stroke=COLORS["line"], stroke_width=3)
        label(c, x + 18, 150, title, size=56)
        c.multiline(x + 18, 207, lines, size=42, leading=48)
    c.polyline(((385, 345), (405, 345)), color=COLORS["blue"], arrow="blue")
    c.polyline(((745, 345), (765, 345)), color=COLORS["blue"], arrow="blue")
    c.polyline(((1095, 345), (1115, 345)), color=COLORS["blue"], arrow="blue")

    c.rect(1730, 30, 650, 500, fill=COLORS["gray"], stroke=COLORS["navy"], stroke_width=4)
    label(c, 1755, 88, "计数口径收敛", size=56)
    c.rect(1755, 112, 570, 42, fill=COLORS["white"], stroke=COLORS["line"])
    c.text(1775, 146, "报警帧｜单时刻命中", size=42)
    c.rect(1785, 164, 510, 42, fill=COLORS["white"], stroke=COLORS["line"])
    c.text(1805, 198, "错误轮次｜测试内命中", size=42)
    c.rect(1815, 216, 450, 42, fill=COLORS["white"], stroke=COLORS["line"])
    c.text(1835, 250, "去重异常签名", size=42)
    c.rect(1845, 268, 390, 42, fill=COLORS["green_fill"], stroke=COLORS["green"])
    c.text(1865, 302, "候选根因族", size=42, fill=COLORS["green"])
    c.rect(1875, 318, 360, 74, fill=COLORS["orange_fill"], stroke=COLORS["orange"])
    c.text(1895, 352, "独立漏洞*", size=42, fill=COLORS["orange"])
    c.text(1895, 386, "*尚未认定", size=42, fill=COLORS["red"], weight="700")
    c.rect(1755, 400, 600, 130, fill=COLORS["white"], stroke=COLORS["gray_text"])
    c.text(1775, 432, "公开Issue 34条", size=42, fill=COLORS["gray_text"])
    c.text(1775, 474, "22 Open / 12 Closed", size=42, fill=COLORS["gray_text"])
    c.text(1775, 516, "非一一对应", size=42, fill=COLORS["red"], weight="700")
    c.polyline(((2050, 400), (2050, 310)), color=COLORS["gray_text"], arrow=None, dash="8 8")
    # A right-side orthogonal rail makes every funnel transition explicit.
    c.polyline(((2325, 133), (2340, 133), (2340, 185), (2295, 185)), color=COLORS["gray_text"], arrow="gray")
    c.polyline(((2295, 185), (2340, 185), (2340, 237), (2265, 237)), color=COLORS["gray_text"], arrow="gray")
    c.polyline(((2265, 237), (2340, 237), (2340, 289), (2235, 289)), color=COLORS["gray_text"], arrow="gray")
    c.polyline(((2235, 289), (2340, 289), (2340, 355), (2235, 355)), color=COLORS["gray_text"], arrow="gray")

    c.rect(1730, 550, 650, 190, fill=COLORS["green_fill"], stroke=COLORS["green"], stroke_width=4)
    label(c, 1755, 590, "证据带", size=56, fill=COLORS["green"])
    c.multiline(1755, 640, ("TB4：928轮→781签名", "→1条加速度路径候选链", "MoveIt：224帧→1个候选族"), size=42, leading=48)

    c.rect(760, 660, 900, 110, fill=COLORS["orange_fill"], stroke=COLORS["orange"], stroke_width=4, dash="14 10")
    label(c, 790, 715, "下一阶段｜尚未开展", size=56, fill=COLORS["orange"])
    c.text(790, 758, "安全种子→Go2/ECO65-B→虚实事件对齐", size=42, fill=COLORS["orange"])
    c.polyline(((1450, 595), (1450, 640), (1210, 640), (1210, 660)), color=COLORS["orange"], arrow="orange", dash="12 10")
    return c.render()
