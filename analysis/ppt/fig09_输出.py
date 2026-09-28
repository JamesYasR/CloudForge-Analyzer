# -*- coding: utf-8 -*-
"""图9：输出与可视化"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svgkit import *

W, H = 1600, 840
sv = SVG(W, H)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../docs/当前需新增功能/ppt_assets/svg")
# ---------------- 左：热力图 ----------------
HX, HY, HW, HH = 90, 170, 560, 420
sv.text(HX, HY - 16, "① 3D 热力图（可切换口径）", size=21, fill=INK, bold=True)
DENTS = [(2.4, 2.0, 1.50), (8.6, 2.6, 0.80), (4.4, 6.4, 0.60), (9.6, 6.9, 0.50)]
RD = 1.75
def dv(c, r):
    v = 0.0
    for (dc, dr, amp) in DENTS:
        dd = math.hypot(c + 0.5 - dc, r + 0.5 - dr)
        if dd < RD:
            v = max(v, amp * (1 - (dd / RD) ** 2))
    return v
NCH, NRH, CWH = 12, 9, 42
for r in range(NRH):
    for c in range(NCH):
        v = dv(c, r)
        t = min(v / 1.5, 1.0)
        if t <= 0:
            fill = "#EFF3F7"
        else:
            rr = int(255 - 60 * (1 - t)); gg = int(255 - 190 * (1 - t)); bb = int(255 - 210 * (1 - t))
            fill = "#%02X%02X%02X" % (rr, gg, bb)
        sv.rect(HX + c * CWH, HY + r * CWH, CWH, CWH, fill=fill, stroke="#E4EAF0", sw=0.8)
sv.ellipse(HX + 2.4 * CWH, HY + 2.0 * CWH, 30, 24, stroke="#0097A7", sw=2.8)
sv.ellipse(HX + 8.6 * CWH, HY + 2.6 * CWH, 22, 18, stroke="#F0B429", sw=2.6)
sv.circle(HX + 2.4 * CWH, HY + 2.0 * CWH, 8, fill="#C2185B")
sv.circle(HX + 4.4 * CWH, HY + 6.4 * CWH, 7, fill="#7B52A1")
sv.text(HX + 4.4 * CWH + 13, HY + 6.4 * CWH + 6, "口径A", size=15, fill="#7B52A1")

# 色条
CBX, CBY, CBW, CBH = HX, HY + HH + 42, 300, 20
for i in range(60):
    t = i / 59.0
    rr = int(255 - 60 * (1 - t)); gg = int(255 - 190 * (1 - t)); bb = int(255 - 210 * (1 - t))
    sv.rect(CBX + i * CBW / 60, CBY, CBW / 60 + 1, CBH, fill="#%02X%02X%02X" % (rr, gg, bb))
sv.rect(CBX, CBY, CBW, CBH, fill="none", stroke=LINE, sw=1.2)
sv.text(CBX, CBY + CBH + 26, "0（周围正常表面）", size=16, fill=SUB)
sv.text(CBX + CBW, CBY + CBH + 26, "下凹越深 →", size=16, fill=SUB, anchor="end")
sv.text(CBX + CBW + 30, CBY + 16, "单位 mm；可切换到“到理想柱面距离 e”", size=17, fill=SUB)
sv.text(HX, CBY + CBH + 60, "椭圆：主坑青色、其余坑黄色（未通过校验的候选坑不画椭圆）",
        size=17, fill=SUB)
sv.text(HX, CBY + CBH + 90, "球标记：主坑洋红、其余坑橙色、未通过校验灰白；两套最深点分别标注",
        size=17, fill=SUB)

# ---------------- 右：报告字段 ----------------
RX, RY, RW, RH = 760, 170, 780, 420
sv.text(RX, RY - 16, "② 报告 / 调试输出", size=21, fill=INK, bold=True)
sv.rect(RX, RY, RW, RH, fill="#FBFCFE", stroke=LINE, sw=1.4, rx=10)
lines = [
    ("共检出 4 个凹塘（通过 4 / 未通过 0）", INK, True),
    ("本次使用参数：W = 90 mm，cell = 1.0 mm，T_local = 0.35 mm", SUB, False),
    ("", SUB, False),
    ("坑 1：深 1.549 mm｜椭圆 28.7 × 20.5 mm｜7686 点｜✅ 通过", RED, False),
    ("坑 2：深 0.919 mm｜椭圆 16.4 × 13.8 mm｜2654 点｜✅ 通过", RED, False),
    ("坑 3：深 0.729 mm｜椭圆 12.5 × 10.0 mm｜1245 点｜✅ 通过", RED, False),
    ("坑 4：深 0.714 mm｜椭圆  9.3 ×  6.1 mm｜ 465 点｜✅ 通过", RED, False),
    ("", SUB, False),
    ("局部基准相对理想柱面偏移：−2.03 ~ +0.88 mm（形面偏差）", PURPLE, False),
    ("边界区被排除比例：48.8%", SUB, False),
    ("口径A（到理想柱面）：最大距离 3.144 mm @ 最深点", BLUE, False),
    ("口径B（局部基准）：最大下凹 1.549 mm @ 最深点", GREEN, False),
]
for i, (t, col, bold) in enumerate(lines):
    sv.text(RX + 28, RY + 46 + i * 30, t, size=18 if not bold else 20, fill=col, bold=bold)
sv.text(RX, RY + RH + 44, "每个坑独立成案：深度、椭圆长短轴、点数、中心、校验结论与拒绝原因。",
        size=18, fill=SUB)
sv.text(RX, RY + RH + 82, "两个“最深点”分别标注（口径A / 口径B），避免把形面偏差误读成坑深。",
        size=18, fill=SUB)
sv.text(RX, RY + RH + 120, "参数与结论一起打印 —— 换参数跑同一数据，报告里能直接看出差异。",
        size=18, fill=SUB)

bad = sv.overflow(2)
print("OVERFLOW:", bad) if bad else print("fig09 no overflow")
sv.strip_title(128, 0)
sv.save(os.path.join(OUT, "fig09_输出与可视化.svg"))
print("fig09 ok")
