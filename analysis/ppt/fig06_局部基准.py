# -*- coding: utf-8 -*-
"""图6：局部基准怎么算 —— 栅格 → 窗口 → 稳健二阶拟合 → 下凹量"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svgkit import *

W, H = 1600, 820
sv = SVG(W, H)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../docs/当前需新增功能/ppt_assets/svg")
GRID = [
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 0, 0, 0, 0],
    [0, 0, 1, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
]
def draw_grid(x, y, cw, ch, vals, disc=False, window=None, mask=None):
    for r, row in enumerate(vals):
        for c, v in enumerate(row):
            fill = WHITE
            if v == 1:
                fill = RED_L
            elif v == 9:
                fill = "#E4E9EE"
            if mask and (r, c) in mask:
                fill = "#FBE3C8"
            sv.rect(x + c * cw, y + r * ch, cw, ch, fill=fill, stroke="#CBD5DE", sw=1.0)
    if window:
        wcx, wcy, wr = window
        sv.circle(x + wcx * cw, y + wcy * ch, wr * cw, fill=BLUE, stroke=BLUE, sw=2.4, op=0.12)

cards = [
    ("① 展开 + 栅格化",
     "每格取中值 m(a,s)\n格边长 ≈ W/90\n（W = 90 → 1 mm 网格）"),
    ("② 以 W/2 为窗口",
     "对每个格，取半径 W/2 内的\n邻近格（W=90 → 半径 45 mm）\n窗口必须足够大：W ≥ 2~3×坑"),
    ("③ 稳健二阶拟合",
     "两轮 3σ 截尾最小二乘\n离群 = 凹坑 / 凸起 / 空白边界\n→ 基准不被坑自身拉偏"),
    ("④ 求下凹量 d = b − m",
     "d > 局部阈值 T（默认 0.35 mm）\n→ 候选点群\n同时给出到理想柱面的距离"),
]
CW, CH, X0, Y0 = 340, 210, 60, 190
for i, (t, cap) in enumerate(cards):
    x = X0 + i * (CW + 30)
    sv.rect(x, Y0 - 34, CW, 500, fill="#FBFCFE", stroke=LINE, sw=1.4, rx=12)
    step_badge(sv, x + 34, Y0 + 2, i + 1, r=17)
    sv.text(x + 60, Y0 + 9, t[1:], size=21, fill=INK, bold=True)
    gx, gy, cw, ch = x + 56, Y0 + 52, 30, 30
    if i == 0:
        draw_grid(gx, gy, cw, ch, GRID)
        sv.text(gx + 4 * cw, gy + 6 * ch + 30, "凹坑所在格", size=16, fill=RED, anchor="middle")
        sv.line(gx + 4 * cw, gy + 6 * ch + 16, gx + 3.6 * cw, gy + 3.2 * ch, stroke=RED, sw=1.3, dash="4 3")
    elif i == 1:
        wcx, wcy = 4.5, 2.5
        sv.circle(gx + wcx * cw, gy + wcy * ch, 2.6 * cw, fill=BLUE, op=0.10, sw=0)
        for r in range(6):
            for c in range(8):
                v = GRID[r][c]
                fill = RED_L if v == 1 else WHITE
                if abs(r - wcy + 0.5) <= 2.6 and abs(c - wcx + 0.5) <= 2.6:
                    fill = "#DCE9F6" if v == 0 else RED_L
                sv.rect(gx + c * cw, gy + r * ch, cw, ch, fill=fill, stroke="#CBD5DE", sw=1.0)
        sv.circle(gx + wcx * cw, gy + wcy * ch, 2.6 * cw, fill="none", stroke=BLUE, sw=2.6)
        sv.circle(gx + wcx * cw, gy + wcy * ch, 4, fill=BLUE)
        sv.text(gx + 6 * cw + 26, gy + 30, "W/2", size=18, fill=BLUE, italic=True)
    elif i == 2:
        px, py, pw, ph = gx + 6, gy + 26, 216, 132
        sv.line(px, py + ph, px + pw, py + ph, stroke=LINE, sw=1.4)
        sv.line(px, py + 10, px, py + ph, stroke=LINE, sw=1.4)
        def bl(u):
            return py + 96 - 0.95 * u * u
        for k in range(17):
            u = k - 8
            sv.circle(px + 12 + k * (pw - 24) / 16.0, bl(u), 4.2, fill=BLUE_M)
        for uo in (-7, 7):
            xx = px + 12 + (uo + 8) * (pw - 24) / 16.0
            yy = bl(uo) + 24
            sv.circle(xx, yy, 5.2, fill=WHITE, stroke=RED, sw=2.2)
            sv.line(xx - 4, yy - 4, xx + 4, yy + 4, stroke=RED, sw=2.0)
            sv.line(xx - 4, yy + 4, xx + 4, yy - 4, stroke=RED, sw=2.0)
        sv.path(catmull_path([(px + 12 + k * (pw - 24) / 40.0, bl(k / 40.0 * 16 - 8))
                              for k in range(41)]), stroke=PURPLE, sw=3.0)
        sv.text(px + pw / 2, py + 16, "拟合 b(a,s)", size=17, fill=PURPLE, anchor="middle")
        sv.text(px + 4, py + ph - 12, "× 离群格：不参与拟合", size=15, fill=RED)
    else:
        px, py, pw, ph = gx + 6, gy + 26, 216, 132
        sv.line(px, py + ph, px + pw, py + ph, stroke=LINE, sw=1.4)
        def bl2(u):
            return py + 96 - 0.95 * u * u
        sv.path(catmull_path([(px + 12 + k * (pw - 24) / 40.0, bl2(k / 40.0 * 16 - 8))
                              for k in range(41)]), stroke=PURPLE, sw=3.0, dash="10 7")
        mp = [(px + 12 + k * (pw - 24) / 40.0,
               bl2(k / 40.0 * 16 - 8) + 30 * math.exp(-((k / 5.0 - 8 + 1.5) ** 2) / 6.0))
              for k in range(41)]
        sv.path(catmull_path(mp), stroke=RED, sw=3.0)
        xk = px + 12 + 16.25 * (pw - 24) / 40.0
        yb, ym = bl2(-1.5), bl2(-1.5) + 30
        sv.line(xk, yb, xk, ym, stroke=GREEN, sw=3.0)
        for yy in (yb, ym):
            sv.line(xk - 9, yy, xk + 9, yy, stroke=GREEN, sw=3.0)
        sv.text(xk - 16, (yb + ym) / 2 + 7, "d", size=21, fill=GREEN, bold=True, italic=True, anchor="end")
        lx = px + pw / 2 - 46
        sv.line(lx, py + 16, lx + 24, py + 16, stroke=PURPLE, sw=3.0, dash="8 5")
        sv.text(lx + 30, py + 22, "b 基准", size=15, fill=PURPLE)
        sv.line(lx, py + 40, lx + 24, py + 40, stroke=RED, sw=3.0)
        sv.text(lx + 30, py + 46, "m 实测", size=15, fill=RED)
    sv.text(x + 22, Y0 + 302, cap, size=17, fill=SUB, lh=26)
    if i < 3:
        sv.path("M%d,%d L%d,%d" % (x + CW + 4, Y0 + 190, x + CW + 26, Y0 + 190),
                stroke=SUB, sw=2.4, marker_end=sv.marker_arrow("arf", SUB, 11))

sv.panel(60, 690, 1480, 96, fill=YELLOW_L, stroke="#E0B93C")
sv.text(90, 730, "基准拟合式：b(a,s) = c₀ + c₁·a + c₂·s + c₃·a² + c₄·s² + c₅·a·s"
                 "（形面偏差以二阶碗形为主，故取二阶）", size=20,
        fill="#7A5B00", bold=True)
sv.text(90, 764, "边界处理：距补丁边界 < W/2 的区域不参与判定（窗口单边会被拉偏）。",
        size=19, fill="#7A5B00")

bad = sv.overflow(2)
print("OVERFLOW:", bad) if bad else print("fig06 no overflow")
sv.strip_title(142, 0)
sv.save(os.path.join(OUT, "fig06_局部基准算法.svg"))
print("fig06 ok")
