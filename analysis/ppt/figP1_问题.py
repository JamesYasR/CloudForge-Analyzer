# -*- coding: utf-8 -*-
"""原理-01 配图：两条基准各自的问题（宽幅 1300x505）"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svgkit import *

W, H = 1300, 505
sv = SVG(W, H)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../docs/当前需新增功能/ppt_assets/svg")

# ================= 左：全局基准 =================
sv.panel(14, 12, 614, 372, fill=WHITE, stroke=RED)
sv.text(38, 48, "① 用设计柱面当基准", size=22, fill=RED, bold=True)
sv.text(300, 48, "形面偏差 3.5 mm ≫ 坑深 1 mm", size=17, fill=SUB)

X0, X1, Y0, Y1 = 112, 596, 78, 290
E_TOP, E_BOT = 2.5, -5.0
def Y(e):
    return Y0 + (E_TOP - e) / (E_TOP - E_BOT) * (Y1 - Y0)
BOWL, TILT = 1.5, 0.5
DENTS = [(-0.40, 0.090, 1.5), (0.267, 0.060, 0.8), (0.60, 0.048, 0.6), (-0.133, 0.032, 0.5)]
def form(t):
    return BOWL * (2 * t * t - 1) + TILT * t
def dev(t):
    v = form(t)
    for c, hw, d in DENTS:
        v -= dent_profile((t - c) / hw, d, 1.0)
    return v

for e in [2, 1, 0, -2, -4]:
    sv.line(X0, Y(e), X1, Y(e), stroke=LINE, sw=1.1)
    sv.text(X0 - 12, Y(e) + 6, f"{e:+d}", size=16, fill=FAINT, anchor="end")
sv.text(X0 - 46, Y0 - 6, "e (mm)", size=17, fill=SUB)
sv.line(X0, Y(0), X1, Y(0), stroke=BLUE, sw=2.6)
sv.text(X0 + 8, Y(0) - 10, "设计柱面 e = 0", size=17, fill=BLUE)
N = 600
ts = [-1 + 2 * i / N for i in range(N + 1)]
sv.path(catmull_path([(X0 + (t + 1) / 2 * (X1 - X0), Y(dev(t))) for t in ts]), stroke=RED, sw=2.6)
THR = -1.06
sv.line(X0, Y(THR), X1, Y(THR), stroke=RED, sw=1.8, dash="11 8")
sv.label_box(X0 + 190, Y(THR) - 22, "判定阈值", size=16, fill=WHITE, stroke=RED, color=RED, bold=True)
vals = [form(t) for t in ts]
runs, st = [], None
for i, v in enumerate(vals):
    if v < THR and st is None:
        st = i
    if (v >= THR or i == N) and st is not None:
        runs.append((ts[st], ts[i])); st = None
runs = [r for r in runs if r[1] - r[0] > 0.005]
for a, b in runs:
    xa, xb = X0 + (a + 1) / 2 * (X1 - X0), X0 + (b + 1) / 2 * (X1 - X0)
    sv.rect(xa, Y0, max(xb - xa, 1.5), Y1 - Y0, fill=RED, op=0.09)
if runs:
    a, b = runs[0]
    xa, xb = X0 + (a + 1) / 2 * (X1 - X0), X0 + (b + 1) / 2 * (X1 - X0)
    sv.label_box((xa + xb) / 2, Y0 + 26, "这一整片都会被当成“凹塘”", size=16,
                 fill=RED_L, stroke=RED, color=RED, bold=True)
TY = Y1 + 24
sv.line(X0, TY, X1, TY, stroke=RED, sw=2.2)
for c, hw, d in DENTS:
    x = X0 + (c + 1) / 2 * (X1 - X0)
    sv.line(x, TY, x, TY - 10, stroke=RED, sw=2.2)
    sv.text(x, TY + 20, f"{d:.1f}", size=15, fill=RED, anchor="middle")
sv.text(X0 - 12, TY + 20, "真实坑深(mm)", size=15, fill=RED, anchor="end")
sv.text(38, 366, "真实缺陷只有 0.5~1.5 mm，完全淹没在形面起伏里。", size=18, fill=INK, bold=True)

# ================= 右：局部三维平面 =================
sv.panel(646, 12, 640, 372, fill=WHITE, stroke=RED)
sv.text(670, 48, "② 用局部三维平面当基准", size=22, fill=RED, bold=True)
cx, cy, Rp = 966, 700, 560
def arc_y(x):
    return cy - math.sqrt(max(Rp * Rp - (x - cx) ** 2, 0.0))
sv.path(catmull_path([(x, arc_y(x)) for x in [660 + i * 610 / 110 for i in range(111)]]),
        stroke=BLUE, sw=3.0)
tx0, tx1 = 726, 1206
yt = arc_y(cx)
sv.line(tx0, yt, tx1, yt, stroke=RED, sw=3.0)
for xe, xm in ((tx0, cx), (tx1, cx)):
    sv.path("M%.1f,%.1f L%.1f,%.1f L%.1f,%.1f Z" % (xe, yt, xm, yt, xe, arc_y(xe)),
            fill=RED, op=0.13)
for xe in (tx0, tx1):
    sv.line(xe, arc_y(xe), xe, yt, stroke=RED, sw=2.4)
    for yy in (arc_y(xe), yt):
        sv.line(xe - 10, yy, xe + 10, yy, stroke=RED, sw=2.4)
sv.label_box(tx1 - 108, yt - 34, "弓高 δ = W²/(8R)", size=17, fill=WHITE, stroke=RED,
             color=RED, bold=True)
sv.line(cx, yt, cx, yt - 40, stroke=SUB, sw=1.4, dash="5 4")
sv.text(cx - 30, yt - 50, "窗口中心（相切点）", size=16, fill=SUB, anchor="end")
sv.text(670, yt + 78, "窗口边缘处，柱面已经离开平面：", size=17, fill=INK)
for i, (ww, dd) in enumerate([(60, 0.23), (90, 0.52), (120, 0.93)]):
    sv.text(670 + i * 200, yt + 116, f"W={ww} → {dd:.2f} mm", size=18,
            fill=(RED if dd > 0.35 else INK), bold=(dd > 0.35))
sv.label_box(966, yt + 200, "R = 1940 mm 时弓高 0.23~0.93 mm，与坑深同量级\n"
                            "→ 基准自己就制造了假缺陷", size=17,
             fill=RED_L, stroke=RED, color="#8C2318")

sv.panel(14, 400, 1272, 92, fill=BLUE_L, stroke=BLUE)
sv.text(40, 442, "两条路都不通：问题不在阈值高低，而在“拿什么当基准”。", size=22,
        fill=BLUE_D, bold=True)
sv.text(40, 474, "一个把零件的形面起伏当成缺陷，另一个把柱面自己的弯曲当成缺陷。", size=18,
        fill=BLUE_D)

bad = sv.overflow(2)
print("OVERFLOW:", bad) if bad else print("figP1 no overflow")
sv.save(os.path.join(OUT, "figP1_问题.svg"))
print("figP1 ok")
