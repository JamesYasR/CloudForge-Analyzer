# -*- coding: utf-8 -*-
"""图2：为什么全局判据失效 —— 形面偏差(2~3mm) 大于坑深(~1mm)"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svgkit import *

W, H = 1600, 900
sv = SVG(W, H)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../docs/当前需新增功能/ppt_assets/svg")
X0, X1 = 190, 1540
Y0, Y1 = 196, 560
E_TOP, E_BOT = 2.5, -4.5

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

for e in [2, 1, 0, -1, -2, -3, -4]:
    sv.line(X0, Y(e), X1, Y(e), stroke="#E6ECF2", sw=1.2)
    sv.text(X0 - 16, Y(e) + 7, f"{e:+d}", size=18, fill=FAINT, anchor="end")
sv.text(X0 - 74, Y0 - 30, "e (mm)", size=20, fill=SUB)
sv.text(X0 + 10, Y0 - 30, "形面偏差峰峰 3.54 mm（−2.0 ~ +0.9 mm）", size=20, fill=ORANGE)
sv.text(X1, Y0 - 30, "沿轴向位置（展开域 a）", size=19, fill=FAINT, anchor="end")

sv.line(X0, Y(0), X1, Y(0), stroke=BLUE, sw=2.8)
sv.text(X0 + 14, Y(0) + 28, "理想柱面 e = 0", size=19, fill=BLUE)

N = 900
ts = [-1 + 2 * i / N for i in range(N + 1)]
sv.path(catmull_path([(X0 + (t + 1) / 2 * (X1 - X0), Y(form(t))) for t in ts]),
        stroke=ORANGE, sw=3.6)
sv.path(catmull_path([(X0 + (t + 1) / 2 * (X1 - X0), Y(dev(t))) for t in ts]),
        stroke=RED, sw=2.8)

THR = -1.06
sv.line(X0, Y(THR), X1, Y(THR), stroke=RED, sw=2.2, dash="13 9")
sv.label_box(X1 - 158, Y(THR) - 30, f"判定阈值 e < {THR:.2f} mm", size=19, fill=WHITE,
             stroke=RED, color=RED, bold=True)

vals = [form(t) for t in ts]
runs, st = [], None
for i, v in enumerate(vals):
    if v < THR and st is None:
        st = i
    if (v >= THR or i == N) and st is not None:
        runs.append((ts[st], ts[i]))
        st = None
runs = [r for r in runs if r[1] - r[0] > 0.005]
for a, b in runs:
    xa = X0 + (a + 1) / 2 * (X1 - X0)
    xb = X0 + (b + 1) / 2 * (X1 - X0)
    sv.rect(xa, Y0 + 4, max(xb - xa, 1.5), Y1 - Y0 - 8, fill=RED, op=0.09)
    sv.line(xa, Y0 + 4, xa, Y1 - 4, stroke=RED, sw=1.2, dash="4 4", op=0.55)

if runs:
    a, b = runs[0]
    xa = X0 + (a + 1) / 2 * (X1 - X0)
    xb = X0 + (b + 1) / 2 * (X1 - X0)
    sv.label_box((xa + xb) / 2, Y0 + 38,
                 "口径A 判定为“凹塘”的区域\n跨越大半个补丁、贴到补丁边界",
                 size=20, fill="#FDF1EF", stroke=RED, color="#8C2318", bold=True)

TY = Y1 + 44
sv.line(X0, TY, X1, TY, stroke=RED, sw=2.6)
for c, hw, d in DENTS:
    x = X0 + (c + 1) / 2 * (X1 - X0)
    sv.line(x, TY, x, TY - 14, stroke=RED, sw=2.6)
    sv.text(x, TY + 30, f"{d:.1f}", size=18, fill=RED, anchor="middle")
sv.text(X0 - 18, TY - 2, "真值坑深\n(mm)", size=18, fill=RED, anchor="end")

legend_row(sv, X0 - 20, 690, [(ORANGE, "口径A 看到的“形面趋势”"), (RED, "叠加 4 个真值凹坑后的表面"),
                              (BLUE, "理想柱面")], size=20, gap=34)
sv.text(X0 - 20, 738, "4 个真值坑：深 1.5 / 0.8 / 0.6 / 0.5 mm，足迹 30×22 / 20×16 / 16×12 / 11×9 mm "
                      "—— 真实缺陷全部落在被误判区域内。", size=20, fill=SUB)
sv.text(X0 - 20, 800, "结论：只要形面偏差 > 坑深，“相对理想柱面”的阈值判据就会把整片形面偏差判成凹塘 —— "
                      "调阈值无法解决。", size=22, fill=INK, bold=True)
sv.text(X0 - 20, 848, "数据来源：仿真样本 pothole_dent_form.pcd（碗形 ±1.5 mm + 倾斜 ±0.5 mm，峰峰 3.537 mm，"
                      "噪声 σ = 0.35 mm）；真实样本 1_cld.pcd 形面偏差 −2.03 ~ +0.88 mm。", size=18, fill=FAINT)

bad = sv.overflow(2)
print("OVERFLOW:", bad) if bad else print("fig02 no overflow")
sv.strip_title(130, 0)
sv.save(os.path.join(OUT, "fig02_难点.svg"))
print("fig02 ok")
