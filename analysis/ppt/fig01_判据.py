# -*- coding: utf-8 -*-
"""图1：凹塘测量的定义与判据（口径A：到理想柱面的距离 e = ρ − R）"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svgkit import *

W, H = 1600, 880
sv = SVG(W, H)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../docs/当前需新增功能/ppt_assets/svg")

PX0, PX1 = 100, 950
BASE, RPX = 560.0, 2100.0
cx = (PX0 + PX1) / 2
cy = BASE + RPX
MM = 27.0

def arc_y(x):
    return cy - math.sqrt(max(RPX * RPX - (x - cx) ** 2, 0.0))

def dev_mm(t):
    return 1.30 * (t * t - 0.5) + 0.50 * t - dent_profile((t + 0.40) / 0.15, 1.0, 1.0)

def surf_y(x):
    t = (x - cx) / ((PX1 - PX0) / 2)
    return arc_y(x) - dev_mm(t) * MM

sv.path(catmull_path([(x, arc_y(x)) for x in
                      [PX0 + i * (PX1 - PX0) / 150 for i in range(151)]]),
        stroke=BLUE, sw=3.6)
sv.path(catmull_path([(x, surf_y(x)) for x in
                      [PX0 + i * (PX1 - PX0) / 320 for i in range(321)]]),
        stroke=RED, sw=3.4)

def e_arrow(x):
    ya, ys = arc_y(x), surf_y(x)
    col = RED if ys > ya else ORANGE          # 曲线在下 → 凹(e<0)；在上 → 凸(e>0)
    sv.line(x, ya, x, ys, stroke=col, sw=2.8)
    for yy in (ya, ys):
        sv.line(x - 13, yy, x + 13, yy, stroke=col, sw=2.8)
    return col, (ya + ys) / 2, (ys > ya)

XZ = cx + 340
col, ym, below = e_arrow(XZ)
sv.label_box(XZ - 10, ym - 62, "e = ρ − R > 0\n（凸：如焊缝余高）", size=18, fill=ORANGE_L,
             stroke=ORANGE, color="#8A4A0B")
XD = cx - 0.40 * ((PX1 - PX0) / 2)
col2, ym2, below2 = e_arrow(XD)
sv.line(XD - 6, surf_y(XD) + 4, XD - 96, 672, stroke=RED, sw=1.5, dash="5 4")
sv.label_box(XD - 118, surf_y(XD) + 80, "e = ρ − R < 0\n（凹：凹塘 / 凹坑）", size=18,
             fill=RED_L, stroke=RED, color="#8C2318")
# 内嵌：整柱截面 + 补丁弧段
ICX, ICY, IR = 258, 214, 104
sv.circle(ICX, ICY, IR, fill="#F7FAFD", stroke=BLUE, sw=2.6)
sv.circle(ICX, ICY, 5, fill=BLUE)
sv.line(ICX, ICY, ICX, ICY - IR, stroke=BLUE, sw=1.6, dash="7 6")
sv.text(ICX + 12, ICY - IR / 2 + 4, "R", size=20, fill=BLUE, italic=True)
a0, a1 = math.radians(-90 - 6.2), math.radians(-90 + 6.2)
sv.path("M%.2f,%.2f A%d,%d 0 0 1 %.2f,%.2f" % (
    ICX + IR * math.cos(a0), ICY + IR * math.sin(a0), IR, IR,
    ICX + IR * math.cos(a1), ICY + IR * math.sin(a1)), stroke=RED, sw=8)
sv.line(ICX + IR * math.cos(a1), ICY + IR * math.sin(a1), 452, 150, stroke=RED, sw=1.5, dash="5 4")
sv.text(462, 144, "扫描补丁：仅 12.35° 弧段", size=20, fill=RED, bold=True)
sv.text(462, 176, "（轴向 261 mm，形面偏差就藏在这里面）", size=18, fill=SUB)
sv.line(ICX, ICY + IR + 8, ICX, BASE - 104, stroke=SUB, sw=1.6, dash="6 6")
sv.text(ICX + 14, BASE - 108, "↓ 该弧段局部放大", size=19, fill=SUB)

print("sign check: 凸点 dev=%.2f mm / 凹点 dev=%.2f mm" % (
    dev_mm((XZ - cx) / ((PX1 - PX0) / 2)), dev_mm((XD - cx) / ((PX1 - PX0) / 2))))

legend_row(sv, PX0 - 10, 764, [(BLUE, "理想柱面（设计半径 R = 1940 mm）"),
                               (RED, "实际扫描表面")], size=20, gap=40)
sv.text(PX0 - 10, 812, "剖面示意：柱面外凸朝上，轴心在下方 1940 mm 处；曲率已放大，径向偏差按 1 mm ≈ 27 px 显示。", size=18, fill=FAINT)
sv.text(PX0 - 10, 858, "凹塘 = 表面相对理想柱面向内偏离、且偏离量超阈值的连通区域。",
        size=21, fill=INK, bold=True)

BX = 1010
sv.panel(BX, 140, 520, 640, fill="#F7F9FC", stroke=LINE)
sv.text(BX + 30, 192, "需求原文的实现链路", size=24, fill=BLUE, bold=True)
for i, s in enumerate(["拼接多片点云，按理想直径拟合圆柱",
                       "计算各点到理想柱面的距离 e = ρ − R",
                       "设定距离阈值，提取偏离点群",
                       "点群轮廓 → 最小二乘椭圆 → 长短轴",
                       "报告最大距离与位置并可视化"]):
    y = 256 + i * 74
    step_badge(sv, BX + 52, y, i + 1, r=18)
    sv.text(BX + 84, y + 9, s, size=20, fill=INK)
sv.line(BX + 30, 648, BX + 490, 648, stroke=LINE, sw=1.4)
sv.text(BX + 30, 692, "隐含前提：形面偏差 ≪ 凹塘深度", size=21, fill=RED, bold=True)
sv.text(BX + 30, 730, "本数据形面偏差反而更大 → 判据失效（见下页）", size=19, fill=SUB)

bad = sv.overflow(2)
print("OVERFLOW:", bad) if bad else print("fig01 no overflow")
sv.strip_title(96, 0)
sv.save(os.path.join(OUT, "fig01_判据.svg"))
print("fig01 ok")
