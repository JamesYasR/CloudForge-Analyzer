# -*- coding: utf-8 -*-
"""原理-04 配图：判定链路 + 边界 + 合理性校验（1300x505）"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svgkit import *

W, H = 1300, 505
sv = SVG(W, H)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../docs/当前需新增功能/ppt_assets/svg")

STEPS = [
    ("算下凹量", "每个格：比周围表面低多少"),
    ("阈值判定", "超过阈值才算候选"),
    ("连通聚类", "分开成一个个独立的坑"),
    ("取轮廓", "取点群的外边界"),
    ("拟合椭圆", "量出长轴与短轴"),
    ("回投与标注", "画回模型上并逐坑编号"),
]
CW, CH, X0, Y0 = 202, 158, 14, 12
for i, (t, d) in enumerate(STEPS):
    x = X0 + i * (CW + 12)
    sv.rect(x, Y0, CW, CH, fill="#FBFCFE", stroke=LINE, sw=1.4, rx=12)
    sv.circle(x + 24, Y0 + 24, 14, fill=BLUE)
    sv.text(x + 24, Y0 + 30, str(i + 1), size=16, fill=WHITE, anchor="middle", bold=True)
    sv.text(x + 44, Y0 + 31, t, size=18, fill=INK, bold=True)
    wrap_arrow(sv, x + 16, Y0 + 72, d, size=15, color=SUB, width=CW - 32)
    if i < 5:
        ax = x + CW + 1
        sv.path("M%d,%d L%d,%d" % (ax, Y0 + CH / 2, ax + 10, Y0 + CH / 2),
                stroke=SUB, sw=2.2, marker_end=sv.marker_arrow("arv", SUB, 10))

# ---------------- 左下：边界 ----------------
sv.panel(14, 182, 640, 310, fill="#FBFCFE", stroke=LINE)
sv.text(38, 218, "边界处理：靠边的地方不判定", size=20, fill=INK, bold=True)
BX, BY, BWD, BHT = 40, 240, 220, 200
sv.rect(BX, BY, BWD, BHT, fill=WHITE, stroke=LINE, sw=1.5)
BW = 34
sv.rect(BX, BY, BWD, BW, fill="#E4E9EE")
sv.rect(BX, BY + BHT - BW, BWD, BW, fill="#E4E9EE")
sv.rect(BX, BY, BW, BHT, fill="#E4E9EE")
sv.rect(BX + BWD - BW, BY, BW, BHT, fill="#E4E9EE")
sv.rect(BX + BW, BY + BW, BWD - 2 * BW, BHT - 2 * BW, fill="none", stroke=GREEN, sw=1.6, dash="8 6")
sv.text(BX + BWD / 2, BY + BHT / 2 + 5, "可判定区", size=16, fill="#1D6B3F", anchor="middle", bold=True)
for (dx, dy, sa, sb) in [(0.32, 0.32, 9, 7), (0.66, 0.30, 7, 5.5), (0.44, 0.68, 6, 4.5),
                         (0.72, 0.70, 4, 3)]:
    x = BX + 0.16 * BWD + dx * 0.68 * BWD
    y = BY + 0.16 * BHT + dy * 0.68 * BHT
    sv.ellipse(x, y, sa, sb, fill=RED, op=0.18)
    sv.ellipse(x, y, sa, sb, stroke=RED, sw=1.8)
ex, ey = BX + 16, BY + 0.55 * BHT
sv.ellipse(ex, ey, 15, 10, fill="#B9C2CB", op=0.5)
sv.ellipse(ex, ey, 15, 10, stroke="#7A8794", sw=1.8, dash="4 3")
sv.line(ex + 16, ey + 8, ex + 34, ey + 34, stroke="#7A8794", sw=1.3, dash="4 3")
sv.text(ex + 38, ey + 40, "贴边坑：不判定", size=15, fill="#5A6672")
wrap_arrow(sv, 286, 254, "距补丁边界不到 W/2 的地方，窗口有一半落在补丁外，基准会被拉偏，"
                         "容易在边上伪造出凹塘。", size=16, color=INK, width=344)
wrap_arrow(sv, 286, 344, "代价：窗口取 90 mm 时，约 48.8% 的面积不参与判定 —— "
                         "贴边的缺陷暂时测不了，这是已知局限。", size=16, color=SUB, width=344)

# ---------------- 右下：四项校验 ----------------
sv.panel(674, 182, 612, 310, fill="#FBFCFE", stroke=LINE)
sv.text(698, 218, "四项合理性校验", size=20, fill=INK, bold=True)
CHECKS = [
    ("贴边", "点群离补丁边界太近"),
    ("面积占比", "点群面积不超过补丁面积的 20%"),
    ("长宽比", "椭圆长轴 / 短轴不超过 5"),
    ("越界", "椭圆不得超出补丁范围"),
]
for i, (t, crit) in enumerate(CHECKS):
    y = 254 + i * 52
    sv.circle(714, y + 8, 14, fill=GREEN)
    sv.text(714, y + 14, str(i + 1), size=15, fill=WHITE, anchor="middle", bold=True)
    sv.text(740, y + 14, t, size=18, fill=INK, bold=True)
    sv.text(830, y + 14, crit, size=16, fill=SUB)
wrap_arrow(sv, 698, 466, "任何一项不过：只报数值，不下“凹塘”的结论。", size=16, color=RED, width=560)

bad = sv.overflow(2)
print("OVERFLOW:", bad) if bad else print("figP4 no overflow")
sv.save(os.path.join(OUT, "figP4_判定与校验.svg"))
print("figP4 ok")
