# -*- coding: utf-8 -*-
"""图4：柱面 → 展开域 (a, s)"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svgkit import *

W, H = 1600, 840
sv = SVG(W, H)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../docs/当前需新增功能/ppt_assets/svg")
CX, CY, R = 350, 430, 200
sv.circle(CX, CY, R, fill="#F7FAFD", stroke=LINE, sw=1.6)
sv.circle(CX, CY, 5, fill=BLUE)
a0, a1 = math.radians(-90 - 22), math.radians(-90 + 22)
sv.path("M%.1f,%.1f A%d,%d 0 0 1 %.1f,%.1f" % (
    CX + R * math.cos(a0), CY + R * math.sin(a0), R, R,
    CX + R * math.cos(a1), CY + R * math.sin(a1)), stroke=RED, sw=10)
for sgn in (-1, 1):
    aa = math.radians(-90 + sgn * 22)
    sv.line(CX, CY, CX + R * math.cos(aa), CY + R * math.sin(aa), stroke=RED, sw=1.6, dash="6 5")
ar = 118
sv.path("M%.1f,%.1f A%d,%d 0 0 1 %.1f,%.1f" % (
    CX + ar * math.cos(a0), CY + ar * math.sin(a0), ar, ar,
    CX + ar * math.cos(a1), CY + ar * math.sin(a1)), stroke=RED, sw=2.0)
sv.text(CX, 268, "φ = 12.35°（示意已放大）", size=20, fill=RED, anchor="middle")
sv.text(CX + 10, CY - R / 2, "R", size=20, fill=BLUE, italic=True)
sv.text(CX, CY + R + 46, "s = R·φ  ∈ [−210, +210] mm", size=20, fill=INK, anchor="middle")
sv.text(CX, CY + R + 80, "弧段很短 → 曲率极小 → 轴线/半径病态", size=18, fill=FAINT, anchor="middle")

sv.path("M650,300 C740,300 762,362 704,398", stroke=SUB, sw=2.4,
        marker_end=sv.marker_arrow("ar", SUB, 13))
sv.text(628, 272, "展开", size=24, fill=SUB, bold=True)
sv.text(628, 304, "（弧长 → 直线）", size=18, fill=SUB)

# ---------------- 右：展开域 ----------------
BX, BY, BW, BH = 800, 160, 700, 480
sv.panel(BX, BY, BW, BH, fill="#FCFDFE", stroke=LINE)
AX, AY = BX + 84, BY + BH - 96
SW, SH = BW - 210, BH - 190
RX0, RY0 = AX + 30, BY + 54
sv.line(AX, AY, AX + SW + 46, AY, stroke=INK, sw=2.2, marker_end=sv.marker_arrow("ay", INK, 12))
sv.line(AX, AY, AX, BY + 30, stroke=INK, sw=2.2, marker_end=sv.marker_arrow("ay", INK, 12))
sv.text(AX + SW + 54, AY + 32, "s（周向弧长, mm）", size=19, fill=SUB, anchor="end")
sv.text(AX - 30, BY + 26, "a（轴向, mm）", size=19, fill=SUB)

sv.rect(RX0, RY0, SW, SH, fill="#F4F8FC", stroke=BLUE_M, sw=1.4)
for i in range(1, 5):
    yy = RY0 + SH * i / 5
    sv.line(RX0, yy, RX0 + SW, yy, stroke=BLUE_M, sw=1.0, dash="5 5", op=0.5)
sv.text(RX0 + SW / 2, RY0 - 16, "理想柱面在展开域 = 平面（e ≡ 0）", size=19, fill=BLUE,
        anchor="middle")
sv.rect(RX0, RY0 + 4, SW, 26, fill="#DDE3E9")
sv.text(RX0 + SW / 2, RY0 + 23, "焊缝区（数据空缺）", size=15, fill=SUB, anchor="middle")
sv.rect(RX0 + 0.60 * SW, RY0 + SH - 32, 0.32 * SW, 22, fill="#DDE3E9")
sv.rect(RX0 + SW - 92, RY0 + 74, 64, 46, fill="#EAEFF4")
sv.text(RX0 + SW - 60, RY0 + 102, "空缺", size=14, fill=SUB, anchor="middle")

sx, sy = SW / 420.0, SH / 300.0
for (a, s, sa, sb, name) in [(-60, 70, 15, 11, "1"), (40, -70, 10, 8, "2"),
                             (90, 60, 8, 6, "3"), (-20, -40, 5.5, 4.5, "4")]:
    x = RX0 + SW * (s + 210) / 420
    y = RY0 + SH * (150 - a) / 300
    sv.ellipse(x, y, max(sb * sx, 4), max(sa * sy, 4), fill=RED, op=0.20)
    sv.ellipse(x, y, max(sb * sx, 4), max(sa * sy, 4), stroke=RED, sw=2.0)
    sv.text(x, y - max(sa * sy, 4) - 8, name, size=16, fill=RED, anchor="middle", bold=True)
sv.text(BX + 30, BY + BH - 34, "4 个凹坑按真实比例画（11~30 mm，相对补丁很小）—— "
                               "在展开域里就是平面上的椭圆。", size=18, fill=RED)

sv.text(BX - 60, BY + BH + 62, "展平后：判断凹塘 = 在平面上找“比局部基准低”的区域 —— 不再受柱面曲率干扰。",
        size=20, fill=INK, bold=True)

bad = sv.overflow(2)
print("OVERFLOW:", bad) if bad else print("fig04 no overflow")
sv.strip_title(116, 0)
sv.save(os.path.join(OUT, "fig04_展开域.svg"))
print("fig04 ok")
