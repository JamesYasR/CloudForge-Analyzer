# -*- coding: utf-8 -*-
"""图8：四道闸门 + 边界排除"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svgkit import *

W, H = 1600, 840
sv = SVG(W, H)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../docs/当前需新增功能/ppt_assets/svg")
# ---------------- 左：展开域补丁 + 边界带 ----------------
PX, PY, PW, PH = 90, 170, 620, 470
sv.rect(PX, PY, PW, PH, fill="#F4F8FC", stroke=BLUE_M, sw=1.6)
BW = 78                                  # W/2 边界带宽度(px)
sv.rect(PX, PY, PW, BW, fill="#E3E8ED")
sv.rect(PX, PY + PH - BW, PW, BW, fill="#E3E8ED")
sv.rect(PX, PY, BW, PH, fill="#E3E8ED")
sv.rect(PX + PW - BW, PY, BW, PH, fill="#E3E8ED")
sv.rect(PX + BW, PY + BW, PW - 2 * BW, PH - 2 * BW, fill="none", stroke=GREEN, sw=2.0, dash="9 7")
sv.text(PX + PW / 2, PY + 44, "边界区：距补丁边界 < W/2 → 不参与判定", size=19, fill=SUB,
        anchor="middle")
sv.text(PX + PW / 2, PY + PH - 20, "实测 W = 90 mm 时，48.8% 的判定格被排除", size=19,
        fill=SUB, anchor="middle")
sv.text(PX + PW / 2, PY + PH / 2 + 8, "可判定区", size=21, fill="#1D6B3F", anchor="middle", bold=True)

for (dx, dy, sa, sb, nm, ok) in [(0.30, 0.34, 15, 11, "1", True), (0.62, 0.30, 10, 8, "2", True),
                                 (0.44, 0.62, 8, 6, "3", True), (0.74, 0.68, 5.5, 4.5, "4", True)]:
    x, y = PX + 0.12 * PW + dx * 0.76 * PW, PY + 0.12 * PH + dy * 0.76 * PH
    sv.ellipse(x, y, sa * 1.15, sb * 1.4, fill=RED, op=0.18)
    sv.ellipse(x, y, sa * 1.15, sb * 1.4, stroke=(RED if ok else "#9AA6B2"), sw=2.2)
    sv.text(x, y - sb * 1.4 - 8, nm, size=15, fill=RED, anchor="middle", bold=True)
# 贴边的一个坑（会被排除）
ex, ey = PX + 0.055 * PW, PY + 0.62 * PH
sv.ellipse(ex, ey, 22, 14, fill="#B9C2CB", op=0.5)
sv.ellipse(ex, ey, 22, 14, stroke="#7A8794", sw=2.2, dash="5 4")
sv.line(ex + 24, ey + 8, ex + 46, ey + 44, stroke="#7A8794", sw=1.5, dash="4 3")
sv.text(ex + 50, ey + 52, "贴边坑：不判定", size=17, fill="#5A6672")

sv.text(90, 686, "为什么必须排除边界：窗口有一半落在补丁外，基准会被系统性拉偏，\n"
                 "在边缘伪造出“凹塘”。", size=18, fill=INK, lh=28)

# ---------------- 右：四道闸门 ----------------
GX, GY, GW, GH = 790, 170, 750, 210
gates = [
    ("① 贴边 / 边界", "点群距补丁边界 < W/2", "由边界排除从根上解决"),
    ("② 点群面积占比", "点群面积 / 补丁面积 ≤ 20%", "整片形面偏差会被这一条挡下"),
    ("③ 椭圆长宽比", "长轴 / 短轴 ≤ 5", "细长条 = 条带状伪缺陷"),
    ("④ 椭圆越界", "椭圆不得越出补丁范围", "局部拟合外推不可信"),
]
for i, (t, crit, why) in enumerate(gates):
    x = GX + (i % 2) * (GW / 2 + 10)
    y = GY + (i // 2) * (GH + 16)
    w = GW / 2 - 10
    sv.rect(x, y, w, GH, fill="#FBFCFE", stroke=LINE, sw=1.4, rx=12)
    step_badge(sv, x + 32, y + 34, i + 1, r=17)
    sv.text(x + 60, y + 41, t[1:], size=21, fill=INK, bold=True)
    sv.text(x + 24, y + 92, crit, size=18, fill=BLUE)
    sv.text(x + 24, y + 130, why, size=17, fill=SUB)
    sv.text(x + 24, y + 172, "不通过 → 不判定为凹塘（仍报告口径A）", size=16, fill=RED)

sv.panel(790, 620, 750, 130, fill=YELLOW_L, stroke="#E0B93C")
sv.text(820, 662, "窗口尺度规则：W ≥ 2~3 × 最大坑尺寸", size=20, fill="#7A5B00", bold=True)
sv.text(820, 698, "W = 60 实测：基准被坑自身拉偏 → 51 个候选簇、大量误报（主坑椭圆 83×45 mm）。",
        size=17, fill="#7A5B00")
sv.text(820, 728, "W = 90（默认）实测：4 个真值坑全部检出、无多余检出。", size=17, fill="#7A5B00")

bad = sv.overflow(2)
print("OVERFLOW:", bad) if bad else print("fig08 no overflow")
sv.strip_title(154, 0)
sv.save(os.path.join(OUT, "fig08_闸门与边界.svg"))
print("fig08 ok")
