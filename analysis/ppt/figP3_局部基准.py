# -*- coding: utf-8 -*-
"""原理-03 配图：局部基准四步（1300x505）"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svgkit import *

W, H = 1300, 505
sv = SVG(W, H)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../docs/当前需新增功能/ppt_assets/svg")

NC, NR, C = 8, 6, 24
GRID = [[0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 0, 0, 0, 0],
        [0, 0, 1, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0]]
cards = [
    ("切成小方格", "把补丁按约 1 mm 切成方格，每格取一个代表值；红色格就是凹坑。"),
    ("看周围一大圈", "对每个方格，取它周围直径 90 mm 内的所有方格，作为这里的正常表面参考。"),
    ("拟合出基准面", "用这一圈方格拟合一个平缓的二次曲面；明显凹下去的方格先剔除，免得把坑自己当成正常表面。"),
    ("量出下凹量", "实测值减去基准面值，就是这里比周围低了多少；超过阈值才算候选。"),
]
CW, CH, X0, Y0 = 306, 388, 14, 12
for i, (t, cap) in enumerate(cards):
    x = X0 + i * (CW + 16)
    sv.rect(x, Y0, CW, CH, fill="#FBFCFE", stroke=LINE, sw=1.4, rx=12)
    sv.circle(x + 26, Y0 + 26, 15, fill=BLUE)
    sv.text(x + 26, Y0 + 33, str(i + 1), size=17, fill=WHITE, anchor="middle", bold=True)
    sv.text(x + 48, Y0 + 33, t, size=20, fill=INK, bold=True)
    gx = x + (CW - NC * C) / 2
    gy = Y0 + 62
    if i == 0:
        for r in range(NR):
            for c in range(NC):
                fill = RED_L if GRID[r][c] else WHITE
                sv.rect(gx + c * C, gy + r * C, C, C, fill=fill, stroke=LINE, sw=1.0)

    elif i == 1:
        wcx, wcy, wr = 3.6, 2.2, 2.7
        for r in range(NR):
            for c in range(NC):
                inw = math.hypot(c + 0.5 - wcx, r + 0.5 - wcy) <= wr
                fill = BLUE_L if inw else WHITE
                if GRID[r][c]:
                    fill = RED_L if inw else RED_L
                sv.rect(gx + c * C, gy + r * C, C, C, fill=fill, stroke=LINE, sw=1.0)
        sv.circle(gx + wcx * C, gy + wcy * C, wr * C, fill="none", stroke=BLUE, sw=2.4)
        sv.circle(gx + wcx * C, gy + wcy * C, 4, fill=BLUE)
        sv.text(gx + NC * C + 2, gy + 18, "90", size=15, fill=BLUE)
    elif i == 2:
        px, py, pw, ph = gx + 4, gy + 16, NC * C - 8, 108
        sv.line(px, py + ph, px + pw, py + ph, stroke=LINE, sw=1.3)
        def bl(u):
            return py + 84 - 0.62 * u * u
        for k in range(15):
            u = k - 7
            sv.circle(px + 10 + k * (pw - 20) / 14.0, bl(u), 3.6, fill=BLUE_M)
        for uo in (-7, 7):
            xx = px + 10 + (uo + 7) * (pw - 20) / 14.0
            yy = bl(uo) + 20
            sv.circle(xx, yy, 4.6, fill=WHITE, stroke=RED, sw=2.0)
            sv.line(xx - 3.6, yy - 3.6, xx + 3.6, yy + 3.6, stroke=RED, sw=1.8)
            sv.line(xx - 3.6, yy + 3.6, xx + 3.6, yy - 3.6, stroke=RED, sw=1.8)
        sv.path(catmull_path([(px + 10 + k * (pw - 20) / 40.0, bl(k / 40.0 * 14 - 7))
                              for k in range(41)]), stroke=BLUE, sw=2.6, dash="9 6")
        sv.text(px + pw, py + 6, "拟合出的基准面", size=15, fill=BLUE, anchor="end")
        sv.text(px + 2, py + ph + 22, "× 先剔除的异常格", size=15, fill=RED)
    else:
        px, py, pw, ph = gx + 4, gy + 16, NC * C - 8, 108
        sv.line(px, py + ph, px + pw, py + ph, stroke=LINE, sw=1.3)
        def bl2(u):
            return py + 84 - 0.62 * u * u
        sv.path(catmull_path([(px + 10 + k * (pw - 20) / 40.0, bl2(k / 40.0 * 14 - 7))
                              for k in range(41)]), stroke=BLUE, sw=2.6, dash="9 6")
        sv.path(catmull_path([(px + 10 + k * (pw - 20) / 40.0,
                               bl2(k / 40.0 * 14 - 7) + 26 * math.exp(
                                   -((k / 40.0 * 14 - 7 + 2.0) ** 2) / 4.5)) for k in range(41)]),
                stroke=RED, sw=2.8)
        xk = px + 10 + 12.0 * (pw - 20) / 40.0
        yb, ym = bl2(-2.0), bl2(-2.0) + 26
        sv.line(xk, yb, xk, ym, stroke=RED, sw=3.0)
        for yy in (yb, ym):
            sv.line(xk - 8, yy, xk + 8, yy, stroke=RED, sw=3.0)
        sv.text(xk + 12, (yb + ym) / 2 + 6, "下凹量", size=15, fill=RED, bold=True)
        sv.text(px + 2, py + ph + 22, "蓝色虚线 = 基准面，红色 = 实测", size=15, fill=SUB)
    wrap_arrow(sv, x + 20, Y0 + 236, cap, size=16, color=SUB, width=266)
    if i < 3:
        ax = x + CW + 2
        sv.path("M%d,%d L%d,%d" % (ax, Y0 + CH / 2, ax + 12, Y0 + CH / 2),
                stroke=SUB, sw=2.4, marker_end=sv.marker_arrow("arp", SUB, 11))

sv.panel(14, 412, 1272, 80, fill=BLUE_L, stroke=BLUE)
sv.text(40, 452, "基准面 b(a,s) = c₀ + c₁a + c₂s + c₃a² + c₄s² + c₅as：形面起伏以“碗形”为主，"
                 "二次曲面足够跟随；再高就会把凹坑也一起跟随掉。", size=19, fill=BLUE_D, bold=True)
sv.text(40, 480, "边界处理：距补丁边界不到 W/2 的地方不参与判定（那里周围没有足够方格可参考）。",
        size=17, fill=BLUE_D)

bad = sv.overflow(2)
print("OVERFLOW:", bad) if bad else print("figP3 no overflow")
sv.save(os.path.join(OUT, "figP3_局部基准.svg"))
print("figP3 ok")
