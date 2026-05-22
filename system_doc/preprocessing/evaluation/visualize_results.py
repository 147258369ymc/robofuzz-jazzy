"""生成预处理流水线评估结果可视化图表"""
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.font_manager import FontProperties
import numpy as np
import os

# 使用 Noto Sans CJK 字体（TTC 中 index=2 为 SC 简体中文）
_FONT_PATH = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
_font = FontProperties(fname=_FONT_PATH)
_font_bold = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc')
matplotlib.rcParams['axes.unicode_minus'] = False

# 评估结果数据
dimensions = ['分块完整性', '归一化准确性', '引用精度', '标签质量', '索引正确性']
scores = [100.0, 100.0, 100.0, 99.2, 95.6]
overall = 98.9
threshold = 90.0

fig, ax = plt.subplots(figsize=(10, 6))

# 绘制柱状图
colors = ['#2ecc71' if s >= threshold else '#e74c3c' for s in scores]
bars = ax.bar(dimensions, scores, color=colors, width=0.6, edgecolor='white', linewidth=1.2)

# 添加阈值线
ax.axhline(y=threshold, color='#e74c3c', linestyle='--', linewidth=1.5, label=f'通过阈值 ({threshold}%)')

# 添加综合得分线
ax.axhline(y=overall, color='#3498db', linestyle='-.', linewidth=1.5, label=f'综合得分 ({overall}%)')

# 在柱子上方标注分数
for bar, score in zip(bars, scores):
    ax.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.5,
            f'{score:.1f}%', ha='center', va='bottom', fontsize=12,
            fontweight='bold', fontproperties=_font_bold)

# 设置坐标轴
ax.set_ylim(0, 108)
ax.set_ylabel('得分 (%)', fontsize=12, fontproperties=_font)
ax.set_title('PX4 规约文档预处理流水线 — 评估结果', fontsize=14,
             fontweight='bold', pad=15, fontproperties=_font_bold)
ax.legend(loc='lower right', fontsize=11, prop=_font)
ax.set_yticks(np.arange(0, 110, 10))
ax.set_xticks(range(len(dimensions)))
ax.set_xticklabels(dimensions, fontproperties=_font, fontsize=11)
ax.grid(axis='y', alpha=0.3)
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

# 添加 PASS 标记
for i, score in enumerate(scores):
    ax.text(i, 5, 'PASS', ha='center', va='bottom', fontsize=10,
            color='white', fontweight='bold', fontproperties=_font_bold)

plt.tight_layout()
output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'evaluation_results.png')
plt.savefig(output_path, dpi=150, bbox_inches='tight')
print(f"图表已保存至: {output_path}")
