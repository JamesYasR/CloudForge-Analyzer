# -*- coding: utf-8 -*-
"""封面配图：理想柱面 / 局部基准 / 实际表面 / 一个凹坑"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svgkit import *

W, H = 1600, 450
sv = SVG(W, H)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../docs/当前需新增功能/ppt_assets/svg")
PX0, PX1 = 60, 1540
BASE, RPX = 180.0, 3000.0
cx = (PX0 + PX1) / 2
cy = BASE + RPX
MM = 34.0

def arc_y(x):
    return cy - math.sqrt(max(RPX * RPX - (x - cx) ** 2, 0.0))

def form(t):
    return 1.3 * (t * t - 0.5) + 0.5 * t

def base_y(x):
    return arc_y(x) - form((x - cx) / ((PX1 - PX0) / 2)) * MM

def surf_y(x):
    t = (x - cx) / ((PX1 - PX0) / 2)
    return base_y(x) + dent_profile((t + 0.34) / 0.15, 1.0, 1.0) * MM

xs = [PX0 + i * (PX1 - PX0) / 200 for i in range(201)]
sv.path(catmull_path([(x, arc_y(x)) for x in xs]), stroke=BLUE, sw=4.0)
sv.path(catmull_path([(x, base_y(x)) for x in xs]), stroke=BLUE, sw=3.4, dash="16 10")
sv.path(catmull_path([(x, surf_y(x)) for x in [PX0 + i * (PX1 - PX0) / 400 for i in range(401)]]),
        stroke=RED, sw=3.6)
XD = cx - 0.34 * ((PX1 - PX0) / 2)
sv.line(XD, base_y(XD), XD, surf_y(XD), stroke=RED, sw=3.6)
for yy in (base_y(XD), surf_y(XD)):
    sv.line(XD - 16, yy, XD + 16, yy, stroke=RED, sw=3.6)
sv.text(XD + 28, (base_y(XD) + surf_y(XD)) / 2 + 10, "d = b − m", size=24, fill=RED, bold=True)
legend_row(sv, PX0, 356, [(BLUE, "理想柱面 e = 0"), (BLUE, "局部基准 b", "dash"),
                          (RED, "实际表面 m")], size=22, gap=46, swatch=(26, 16))
sv.text(PX0, 414, "判断用局部基准，报告给理想柱面下的距离",
        size=21, fill=SUB)

bad = sv.overflow(2)
print("OVERFLOW:", bad) if bad else print("fig00 no overflow")
sv.save(os.path.join(OUT, "fig00_封面.svg"))
print("fig00 ok")
