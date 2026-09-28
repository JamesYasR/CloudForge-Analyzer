# -*- coding: utf-8 -*-
"""图3：两个口径分工 —— 口径A 报告、口径B 判断"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svgkit import *

W, H = 1600, 900
sv = SVG(W, H)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../docs/当前需新增功能/ppt_assets/svg")
PX0, PX1 = 110, 950
BASE, RPX = 500.0, 2200.0
cx = (PX0 + PX1) / 2
cy = BASE + RPX
MM = 40.0
DC, DHW = -0.55, 0.30          # 凹坑中心/半宽（展开域归一化）

def arc_y(x):
    return cy - math.sqrt(max(RPX * RPX - (x - cx) ** 2, 0.0))

def form(t):
    return 2.0 * (t * t - 0.5) + 0.60 * t

def dent(t):
    return dent_profile((t - DC) / DHW, 1.0, 1.0, flat=0.5)

def base_y(x):
    return arc_y(x) - form((x - cx) / ((PX1 - PX0) / 2)) * MM

def surf_y(x):
    t = (x - cx) / ((PX1 - PX0) / 2)
    return arc_y(x) - (form(t) - dent(t)) * MM

# ---------- 顶部：形面偏差从哪来 ----------
sv.panel(110, 122, 400, 236, fill="#FBFCFE", stroke=LINE)
sv.rect(150, 156, 320, 148, fill="#FDF3E7", stroke=ORANGE, sw=1.6, rx=6)
for i in range(7):
    k = 1 - i / 7.0
    sv.circle(310, 230, 68 * k, stroke=ORANGE, sw=1.5, op=0.85)
sv.circle(310, 230, 5, fill=ORANGE)
sv.text(310, 322, "补丁上的形面偏差：跨整个补丁的平滑起伏（峰峰 2~3 mm）", size=17,
        fill="#8A4A0B", anchor="middle")
sv.text(150, 142, "① 形面偏差长什么样", size=19, fill=ORANGE, bold=True)

sv.panel(540, 122, 410, 236, fill="#FBFCFE", stroke=LINE)
sv.text(566, 158, "② 两条基准从哪来", size=19, fill=INK, bold=True)
sv.text(566, 200, "口径A：由设计半径 R = 1940 mm 直接给出", size=18, fill=BLUE)
sv.text(566, 236, "口径B：在“周围正常表面”上就地拟合", size=18, fill=PURPLE)
sv.text(566, 272, "两者之差 = 该处的形面偏差", size=18, fill=ORANGE)
sv.text(566, 310, "→ 判断只用口径B，报告仍给口径A", size=18, fill=INK, bold=True)
sv.text(566, 342, "（两者都必须给，缺一不可）", size=17, fill=FAINT)

xs = [PX0 + i * (PX1 - PX0) / 150 for i in range(151)]
sv.path(catmull_path([(x, arc_y(x)) for x in xs]), stroke=BLUE, sw=3.6)
sv.path(catmull_path([(x, base_y(x)) for x in xs]), stroke=PURPLE, sw=3.2, dash="14 8")
sv.path(catmull_path([(x, surf_y(x)) for x in [PX0 + i * (PX1 - PX0) / 360 for i in range(361)]]),
        stroke=RED, sw=3.2)

def darrow(x, y1, y2, color, sw=3.0):
    sv.line(x, y1, x, y2, stroke=color, sw=sw)
    for yy in (y1, y2):
        sv.line(x - 14, yy, x + 14, yy, stroke=color, sw=sw)

# 口径A：蓝 → 红（含形面偏差）
XA = 250
darrow(XA, arc_y(XA), surf_y(XA), BLUE)
sv.label_box(XA + 118, arc_y(XA) - 60, "口径A   e = ρ − R\n（把形面偏差一起算进去 → 偏大甚至失效）",
             size=18, fill=BLUE_L, stroke=BLUE, color="#12456F")

# 口径B：紫 → 红（真实坑深）
XB = 316
darrow(XB, base_y(XB), surf_y(XB), PURPLE, sw=3.6)
sv.label_box(XB + 190, surf_y(XB) + 86, "口径B   d = b − m\n（只量“比周围低了多少” = 真实坑深）",
             size=18, fill=PURPLE_L, stroke=PURPLE, color="#4A2E6B")

sv.text(PX0 - 10, 792, "同一个坑、同一处位置：口径A 的箭头里混进了形面偏差，口径B 的箭头才是坑本身。",
        size=20, fill=INK, bold=True)
legend_row(sv, PX0 - 10, 842, [(BLUE, "理想柱面 e = 0"), (PURPLE, "局部基准 b（周围正常表面）"),
                               (RED, "实际表面 m")], size=19, gap=32)

# ---------------- 右：分工表 ----------------
BX, BY, BW = 1020, 140, 520
sv.panel(BX, BY, BW, 296, fill=GREEN_L, stroke=GREEN)
sv.text(BX + 26, BY + 46, "口径B：局部基准（判断用）", size=23, fill="#1D6B3F", bold=True)
sv.text(BX + 26, BY + 92, "d = b − m", size=26, fill=INK, bold=True)
for i, s in enumerate(["阈值判定：d > T_local", "提取点群 → 聚类",
                       "轮廓 → 椭圆 → 长短轴", "四道合理性闸门"]):
    sv.text(BX + 26, BY + 140 + i * 36, "· " + s, size=19, fill=INK)

sv.panel(BX, BY + 320, BW, 296, fill=BLUE_L, stroke=BLUE)
sv.text(BX + 26, BY + 366, "口径A：理想柱面（报告用）", size=23, fill="#12456F", bold=True)
sv.text(BX + 26, BY + 412, "e = ρ − R", size=26, fill=INK, bold=True)
for i, s in enumerate(["需求原文要求的口径", "超差判定 / 最大距离与位置",
                       "建立轴向—周向—径向坐标系", "与历史数据可比"]):
    sv.text(BX + 26, BY + 460 + i * 36, "· " + s, size=19, fill=INK)

sv.panel(BX, BY + 640, BW, 104, fill=YELLOW_L, stroke="#E0B93C")
sv.text(BX + 26, BY + 678, "结果同时给出两个数：局部深度 d 与到理想柱面距离 e", size=19,
        fill="#7A5B00", bold=True)
sv.text(BX + 26, BY + 710, "判定结论只由口径B 产生，避免把形面偏差当缺陷。", size=19, fill="#7A5B00")

bad = sv.overflow(2)
print("OVERFLOW:", bad) if bad else print("fig03 no overflow")
sv.strip_title(104, 0)
sv.save(os.path.join(OUT, "fig03_两个口径.svg"))
print("fig03 ok")
