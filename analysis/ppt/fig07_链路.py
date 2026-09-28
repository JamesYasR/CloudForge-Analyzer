# -*- coding: utf-8 -*-
"""图7：判定链路 —— 下凹量 → 阈值 → 聚类 → 轮廓 → 椭圆 → 回投标注"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svgkit import *

W, H = 1600, 860
sv = SVG(W, H)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../docs/当前需新增功能/ppt_assets/svg")
NC, NR = 12, 9
CW = CH = 20
BLUE_T, GRAY_T = "#EAF2FB", "#F7F9FB"
DENTS = [(3.0, 2.4, 1.50, "1"), (8.4, 3.4, 0.80, "2"), (5.6, 6.2, 0.60, "3"), (9.4, 6.8, 0.50, "4")]
R0 = 2.7
def dval(c, r):
    v = 0.0
    for (dc, dr, amp, _) in DENTS:
        dd = math.hypot(c + 0.5 - dc, r + 0.5 - dr)
        if dd < R0:
            v = max(v, amp * (1 - (dd / R0) ** 2))
    return v

def hull(pts):
    pts = sorted(set(pts))
    if len(pts) <= 2:
        return pts
    def half(ps):
        out = []
        for p in ps:
            while len(out) >= 2 and ((out[-1][0] - out[-2][0]) * (p[1] - out[-2][1])
                                     - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(p)
        return out
    lo = half(pts)
    up = half(list(reversed(pts)))
    return lo[:-1] + up[:-1]

CARDW, CARDH, X0, Y0 = 232, 470, 40, 176
MARGIN = (1600 - 40 - 6 * CARDW) / 5.0

for i in range(6):
    x = X0 + i * (CARDW + MARGIN)
    sv.rect(x, Y0, CARDW, CARDH, fill="#FBFCFE", stroke=LINE, sw=1.4, rx=12)
    sv.circle(x + 26, Y0 + 26, 15, fill=BLUE)
    sv.text(x + 26, Y0 + 33, str(i + 1), size=18, fill=WHITE, anchor="middle", bold=True)
    gx, gy = x + 26, Y0 + 56

    if i == 0:
        sv.text(x + 46, Y0 + 33, "算下凹量 d", size=19, fill=INK, bold=True)
        for r in range(NR):
            for c in range(NC):
                v = dval(c, r)
                fill = WHITE if v <= 0 else RED_L
                sv.rect(gx + c * CW, gy + r * CH, CW, CH, fill=fill, stroke="#DEE5EC", sw=1.0)
        sv.text(x + CARDW / 2, gy + NR * CH + 46, "每个格：d = b − m\n（口径B，相对周围表面）", size=16,
                fill=SUB, anchor="middle")
    elif i == 1:
        sv.text(x + 46, Y0 + 33, "阈值判定", size=19, fill=INK, bold=True)
        for r in range(NR):
            for c in range(NC):
                v = dval(c, r)
                fill = RED_L if v > 0.35 else WHITE
                if 0 < v <= 0.35:
                    fill = "#FDF3E7"
                sv.rect(gx + c * CW, gy + r * CH, CW, CH, fill=fill, stroke="#DEE5EC", sw=1.0)
        sv.text(x + CARDW / 2, gy + NR * CH + 46, "d > T_local（0.35 mm）\n→ 候选格（红）", size=16,
                fill=SUB, anchor="middle")
    elif i == 2:
        sv.text(x + 46, Y0 + 33, "连通聚类", size=19, fill=INK, bold=True)
        cols = {"1": "#E8B4AE", "2": "#BFD9F2", "3": "#F5D6A8", "4": "#CDE7D3"}
        def which(c, r):
            best, bd = None, 9e9
            for (dc, dr, amp, nm) in DENTS:
                dd = math.hypot(c + 0.5 - dc, r + 0.5 - dr)
                if dd < R0 and dd < bd:
                    best, bd = nm, dd
            return best
        for r in range(NR):
            for c in range(NC):
                nm = which(c, r)
                fill = cols[nm] if nm else WHITE
                sv.rect(gx + c * CW, gy + r * CH, CW, CH, fill=fill, stroke="#DEE5EC", sw=1.0)
        for (dc, dr, amp, nm) in DENTS:
            sv.text(gx + dc * CW, gy + dr * CH + 6, nm, size=16,
                    fill=INK, anchor="middle", bold=True)
        sv.text(x + CARDW / 2, gy + NR * CH + 46, "4 个独立点群（逐坑独立成案）\n不会把两个坑连成一个", size=16,
                fill=SUB, anchor="middle")
    elif i == 3:
        sv.text(x + 46, Y0 + 33, "取轮廓（凸包）", size=19, fill=INK, bold=True)
        cx0, cy0 = gx + 4.0 * CW, gy + 4.2 * CH
        inside = []
        for r in range(NR):
            for c in range(NC):
                hit = math.hypot(c + 0.5 - 4.0, (r + 0.5 - 4.2) / 0.86) < 2.45
                if hit:
                    inside.append((c, r))
                sv.rect(gx + c * CW, gy + r * CH, CW, CH, fill=(RED_L if hit else WHITE),
                        stroke="#DEE5EC", sw=1.0)
        hp = hull([(gx + (c + 0.5) * CW, gy + (r + 0.5) * CH) for c, r in inside])
        sv.poly(hp, fill=RED, stroke=RED, sw=2.8, op=0.13)
        sv.poly(hp, stroke=RED, sw=2.8)
        sv.text(x + CARDW / 2, gy + NR * CH + 46, "点群凸包当轮廓\n（栅格边界会内缩约 1 格）", size=16,
                fill=SUB, anchor="middle")
    elif i == 4:
        sv.text(x + 46, Y0 + 33, "拟合椭圆", size=19, fill=INK, bold=True)
        cx0, cy0 = gx + 4.0 * CW, gy + 4.2 * CH
        sv.ellipse(cx0, cy0, 2.5 * CW, 2.0 * CH, fill=BLUE_L, stroke=BLUE, sw=2.6)
        sv.line(cx0 - 2.5 * CW, cy0, cx0 + 2.5 * CW, cy0, stroke=BLUE, sw=2.0)
        sv.line(cx0, cy0 - 2.0 * CH, cx0, cy0 + 2.0 * CH, stroke=BLUE, sw=2.0)
        sv.text(cx0, cy0 - 2.0 * CH - 10, "长轴 = 2a", size=16, fill=BLUE, anchor="middle")
        sv.text(cx0 + 2.5 * CW + 8, cy0 + 6, "短轴\n= 2b", size=16, fill=BLUE)
        sv.text(x + CARDW / 2, gy + NR * CH + 46, "直接最小二乘（Halir-Flusser）\n长短轴单位 mm", size=16,
                fill=SUB, anchor="middle")
    else:
        sv.text(x + 46, Y0 + 33, "回投与标注", size=19, fill=INK, bold=True)
        cx0, cy0 = gx + 5.0 * CW, gy + 4.5 * CH
        sv.path("M%d,%d Q%d,%d %d,%d" % (cx0 - 90, cy0 + 26, cx0, cy0 - 80, cx0 + 90, cy0 + 26),
                stroke=BLUE_M, sw=2.4, dash="7 6")
        sv.ellipse(cx0, cy0 + 4, 46, 18, fill=RED, stroke=RED, sw=2.4, op=0.22)
        sv.ellipse(cx0, cy0 + 4, 46, 18, stroke=RED, sw=2.6)
        sv.circle(cx0, cy0 + 4, 5.5, fill=MAGENTA if False else "#C2185B")
        sv.text(cx0 - 4, cy0 + 52, "最深点", size=16, fill="#C2185B", anchor="middle")
        sv.text(x + CARDW / 2, gy + NR * CH + 46, "椭圆回投柱面 + 逐坑编号\n颜色区分主坑/其余/未通过", size=16,
                fill=SUB, anchor="middle")

    if i < 5:
        ax = x + CARDW + 4
        sv.path("M%d,%d L%d,%d" % (ax, Y0 + CARDH / 2, ax + MARGIN - 8, Y0 + CARDH / 2),
                stroke=SUB, sw=2.6, marker_end=sv.marker_arrow("arx", SUB, 12))

sv.panel(40, 690, 1520, 128, fill="#F7F9FC", stroke=LINE)
sv.text(70, 730, "全程确定性算法（无 RANSAC 随机性）：同一份数据重复运行，结果逐位一致。",
        size=20, fill=INK, bold=True)
sv.text(70, 766, "多个凹塘时：按点数降序逐坑独立成案，报告主坑（点数最多）与全部候选，"
                 "未通过闸门的也列出并说明原因。", size=19, fill=SUB)
sv.text(70, 800, "同时输出：口径A 的最大距离/最深点（需求原文）＋ 口径B 的局部深度/最深点 —— 两套数都给。",
        size=19, fill=SUB)

bad = sv.overflow(2)
print("OVERFLOW:", bad) if bad else print("fig07 no overflow")
sv.strip_title(162, 0)
sv.save(os.path.join(OUT, "fig07_判定链路.svg"))
print("fig07 ok")
