# -*- coding: utf-8 -*-
"""图5：为什么基准必须"在展开域里"建"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svgkit import *

W, H = 1600, 800
sv = SVG(W, H)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../docs/当前需新增功能/ppt_assets/svg")
# ---------------- 左：3D 切平面（不可用） ----------------
sv.panel(90, 150, 660, 560, fill="#FDF6F5", stroke=RED)
sv.text(122, 196, "✗  用 3D 切平面当基准", size=24, fill=RED, bold=True)
sv.text(560, 196, "（曲率示意放大）", size=17, fill=FAINT)
cx, cy, Rp = 420, 950, 620
def arc_y(x):
    return cy - math.sqrt(max(Rp * Rp - (x - cx) ** 2, 0.0))
xs = [150 + i * 540 / 140 for i in range(141)]
sv.path(catmull_path([(x, arc_y(x)) for x in xs]), stroke=BLUE, sw=3.6)
tx0, tx1 = 190, 650
yt = arc_y(cx)
sv.line(tx0, yt, tx1, yt, stroke=ORANGE, sw=3.2)
for xe, xm in ((tx0, cx), (tx1, cx)):
    sv.path("M%.1f,%.1f L%.1f,%.1f L%.1f,%.1f Z" % (xe, yt, xm, yt, xe, arc_y(xe)),
            fill=ORANGE, op=0.18)
for xe in (tx0, tx1):
    sv.line(xe, arc_y(xe), xe, yt, stroke=RED, sw=2.6)
    for yy in (arc_y(xe), yt):
        sv.line(xe - 12, yy, xe + 12, yy, stroke=RED, sw=2.6)
sv.label_box(tx1 - 118, yt - 40, "弓高 δ = W²/(8R)", size=19, fill=WHITE, stroke=RED,
             color=RED, bold=True)
sv.line(cx, yt, cx, yt - 48, stroke=SUB, sw=1.5, dash="6 5")
sv.text(cx, yt - 60, "窗口中心（相切点）", size=18, fill=SUB, anchor="middle")
sv.text(122, yt + 92, "切平面与柱面在中心相切，窗口边缘处差出弓高：", size=19, fill=INK)
sv.text(122, yt + 134, "δ = W² / (8R)", size=24, fill=RED, bold=True)
sv.text(122, yt + 180, "R = 1940 mm 时：", size=19, fill=SUB)
for i, (ww, dd) in enumerate([(60, 0.23), (90, 0.52), (120, 0.93), (150, 1.45)]):
    xx = 122 + (i % 2) * 250
    yy = yt + 220 + (i // 2) * 36
    sv.text(xx, yy, f"W = {ww:>3d} mm → {dd:.2f} mm", size=19,
            fill=(RED if dd > 0.35 else INK), bold=(dd > 0.35))
sv.label_box(420, yt + 302, "弓高（0.23~1.45 mm）与坑深（0.5~1.5 mm）同量级\n"
                            "→ 基准自身就歪，直接产生假凹塘", size=18,
             fill=RED_L, stroke=RED, color="#8C2318")

# ---------------- 右：展开域（可用） ----------------
sv.panel(790, 150, 720, 560, fill="#F4FAF6", stroke=GREEN)
sv.text(822, 196, "✓  在展开域 (a, s) 里建基准", size=24, fill="#1D6B3F", bold=True)
sv.line(850, 470, 1470, 470, stroke=BLUE, sw=3.4)
sv.text(850, 508, "理想柱面 → 直线（展开后曲率 = 0）", size=19, fill=BLUE)
def oy(x):
    return 434 - 34 * math.cos((x - 1160) / 620 * 3.14159)
xs2 = [850 + i * 620 / 100 for i in range(101)]
sv.path(catmull_path([(x, oy(x)) for x in xs2]), stroke=ORANGE, sw=3.2, dash="12 8")
sv.text(850, 356, "二阶局部基准 b(a,s)（跟随碗形/倾斜）", size=19, fill=ORANGE)
sv.line(1240, 470, 1240, oy(1240), stroke=ORANGE, sw=2.6)
for yy in (470, oy(1240)):
    sv.line(1226, yy, 1254, yy, stroke=ORANGE, sw=2.6)
sv.text(1258, (470 + oy(1240)) / 2 + 6, "差值 = 形面偏差（低阶）", size=18, fill=SUB)
sv.text(850, 268, "展开域里的二阶项 (a², s², a·s) 对应 3D 中的", size=18, fill=SUB)
sv.text(850, 300, "“局部半径略变 / 鼓形 / 锥度”，不引入柱面曲率。", size=18, fill=SUB)
sv.text(850, 560, "弓高偏差 ≡ 0", size=26, fill=GREEN, bold=True)
sv.text(850, 600, "曲面基准只承担“跟随形面”，不承担“跟随柱面曲率”。", size=19, fill=INK)
sv.text(850, 640, "→ 阈值 T_local = 0.35 mm 才有物理意义。", size=19, fill=INK)
sv.label_box(1160, 690, "结论：不是“用不用曲面基准”，而是“在哪个域里建”。", size=19,
             fill=WHITE, stroke=GREEN, color="#1D6B3F")

bad = sv.overflow(2)
print("OVERFLOW:", bad) if bad else print("fig05 no overflow")
sv.strip_title(136, 0)
sv.save(os.path.join(OUT, "fig05_弓高.svg"))
print("fig05 ok")
