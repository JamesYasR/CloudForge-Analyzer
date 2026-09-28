# -*- coding: utf-8 -*-
"""原理-02 配图：摊平成平面图 + 局部基准/设计柱面分工（1300x505）"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svgkit import *

W, H = 1300, 505
sv = SVG(W, H)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../docs/当前需新增功能/ppt_assets/svg")

# ---------------- 左：摊平 ----------------
sv.panel(14, 12, 412, 372, fill=GRAY_L, stroke=LINE)
sv.text(36, 48, "① 把柱面摊平成平面图", size=21, fill=INK, bold=True)
sv.text(36, 78, "a = 轴向，s = 周向弧长", size=16, fill=SUB)
CX, CY, R = 220, 190, 70
sv.circle(CX, CY, R, fill=WHITE, stroke=LINE, sw=1.5)
sv.circle(CX, CY, 4, fill=BLUE)
a0, a1 = math.radians(-90 - 24), math.radians(-90 + 24)
sv.path("M%.1f,%.1f A%d,%d 0 0 1 %.1f,%.1f" % (
    CX + R * math.cos(a0), CY + R * math.sin(a0), R, R,
    CX + R * math.cos(a1), CY + R * math.sin(a1)), stroke=RED, sw=7)
for sgn in (-1, 1):
    aa = math.radians(-90 + sgn * 24)
    sv.line(CX, CY, CX + R * math.cos(aa), CY + R * math.sin(aa), stroke=RED, sw=1.3, dash="5 4")
sv.text(CX, 110, "扫描补丁", size=16, fill=RED, anchor="middle")
sv.text(CX, 288, "按弧长摊开 ↓", size=17, fill=SUB, anchor="middle")
sv.path("M%d,%d L%d,%d" % (CX, 298, CX, 328), stroke=SUB, sw=2.4,
        marker_end=sv.marker_arrow("ap", SUB, 12))
sv.rect(96, 338, 248, 34, fill=WHITE, stroke=BLUE, sw=1.4, rx=4)
for i in range(1, 4):
    sv.line(96 + 248 * i / 4, 338, 96 + 248 * i / 4, 372, stroke=BLUE, sw=0.9, dash="4 4", op=0.5)
sv.text(220, 361, "平面图 (a, s)：柱面 = 平面", size=16, fill=BLUE, anchor="middle")

# ---------------- 右：两条基准 ----------------
sv.panel(446, 12, 840, 372, fill="#FBFCFE", stroke=LINE)
sv.text(470, 48, "② 在平面图上，用“周围正常表面”当基准", size=21, fill=INK, bold=True)
PX0, PX1 = 500, 1240
BASE, RPX = 240.0, 1500.0
cx = (PX0 + PX1) / 2
cy = BASE + RPX
MM = 22.0
DC, DHW = -0.55, 0.30

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

xs = [PX0 + i * (PX1 - PX0) / 140 for i in range(141)]
sv.path(catmull_path([(x, arc_y(x)) for x in xs]), stroke=BLUE, sw=3.0)
sv.path(catmull_path([(x, base_y(x)) for x in xs]), stroke=BLUE, sw=3.0, dash="13 8")
sv.path(catmull_path([(x, surf_y(x)) for x in [PX0 + i * (PX1 - PX0) / 300 for i in range(301)]]),
        stroke=RED, sw=3.0)

XB = 590
sv.line(XB, base_y(XB), XB, surf_y(XB), stroke=RED, sw=3.4)
for yy in (base_y(XB), surf_y(XB)):
    sv.line(XB - 12, yy, XB + 12, yy, stroke=RED, sw=3.4)
sv.label_box(XB + 116, surf_y(XB) + 50, "判断：比周围表面低多少", size=17, fill=RED_L,
             stroke=RED, color=RED)
XA = 830
sv.line(XA, arc_y(XA), XA, surf_y(XA), stroke=BLUE, sw=2.8)
for yy in (arc_y(XA), surf_y(XA)):
    sv.line(XA - 12, yy, XA + 12, yy, stroke=BLUE, sw=2.8)
sv.label_box(XA + 130, arc_y(XA) - 40, "报告：到设计柱面的距离", size=17, fill=BLUE_L,
             stroke=BLUE, color=BLUE_D)
legend_row(sv, 806, 92, [(BLUE, "设计柱面"), (BLUE, "局部基准 b", "dash"),
                          (RED, "实测表面")], size=16, gap=20)

sv.panel(14, 400, 1272, 92, fill=BLUE_L, stroke=BLUE)
sv.text(40, 438, "判断用局部基准，对外报告仍按设计柱面 —— 两个数都不缺。", size=21,
        fill=BLUE_D, bold=True)
sv.text(40, 472, "圆柱面和纸一样属于“可展曲面”：摊平不拉伸、不撕裂，摊平前后量到的长度完全一样；"
                 "理想柱面摊平后就是平面，所以局部基准只要跟随零件起伏，不必再去跟随柱面弯曲。",
        size=16, fill=BLUE_D)

bad = sv.overflow(2)
print("OVERFLOW:", bad) if bad else print("figP2 no overflow")
sv.save(os.path.join(OUT, "figP2_思路.svg"))
print("figP2 ok")
