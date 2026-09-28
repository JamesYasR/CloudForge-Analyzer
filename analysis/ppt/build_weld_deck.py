# -*- coding: utf-8 -*-
"""生成《焊前装配测量 —— 思路、难点与操作流程》PPTX（给甲方看，11 页）。

⚠️ 注意：docs/当前需新增功能/ 下的 PPTX 已由用户手工修改过。重跑本脚本会**覆盖手工修改**，
   除非用户明确要求重出，不要运行它。


风格约束：只用蓝/红/灰；全部直角矩形（无圆角、无装饰背景）；
每页正文不超过两段；原理页 = 标题 + 一句总起 + 一张 SVG；
截图页用占位框。调试类文字与口径讨论一律不出现。
"""
import os, sys, subprocess, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import svgkit as sv
import deck_kit as dk
from pptx import Presentation
from pptx.util import Inches
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.dml import MSO_LINE_DASH_STYLE

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../.."))
SVD = os.path.join(ROOT, "docs/当前需新增功能/ppt_assets/svg")
PNG = os.path.join(ROOT, "docs/当前需新增功能/ppt_assets/png")
OUT = os.path.join(ROOT, "docs/当前需新增功能/焊前装配测量-思路与流程.pptx")
os.makedirs(SVD, exist_ok=True)
os.makedirs(PNG, exist_ok=True)

FW, FH = 1300, 505          # 原理图尺寸


# ============================================================ SVG 图
def fig_link():
    """总体思路：六步链路 + 一页说明。"""
    s = sv.SVG(FW, FH)
    steps = [("1", "选圆柱基准", "已保存的圆柱结果"),
             ("2", "选远件 / 近件", "两份点云"),
             ("3", "展开成平面图", "轴向 × 周向弧长"),
             ("4", "提取缝边", "两件的接缝边"),
             ("5", "沿轴向配对", "一对点"),
             ("6", "出两个量", "阶差 + 间隙")]
    n = len(steps)
    gap, x0, y0 = 24, 34, 112
    bw = (FW - 2 * x0 - (n - 1) * gap) / n
    bh = 152
    for i, (num, t, sub) in enumerate(steps):
        x = x0 + i * (bw + gap)
        s.rect(x, y0, bw, bh, fill=sv.WHITE, stroke=sv.LINE, sw=1.4)
        s.rect(x, y0, bw, 5, fill=sv.BLUE)
        sv.step_badge(s, x + bw / 2, y0 + 40, num)
        s.text(x + bw / 2, y0 + 80, t, size=19, fill=sv.INK, bold=True, anchor="middle")
        s.text(x + bw / 2, y0 + 116, sub, size=15, fill=sv.SUB, anchor="middle")
        if i < n - 1:
            s.line(x + bw + 3, y0 + bh / 2, x + bw + gap - 3, y0 + bh / 2,
                   stroke=sv.BLUE, sw=2.2, marker_end="arrow")
    s.rect(x0, 322, FW - 2 * x0, 130, fill=sv.BLUE_L, stroke="none")
    s.text(x0 + 28, 362, "基准来自圆柱功能：本功能不再拟合圆柱，只选择并使用已保存的结果。",
           size=20, fill=sv.BLUE_D)
    s.text(x0 + 28, 408, "全部参数自动推导；配对只沿轴向进行，阶差与间隙由同一次配对同时得到。",
           size=20, fill=sv.INK)
    s.save(os.path.join(SVD, "weld_f1_链路.svg"))


def fig_base():
    """难点①：大半径小弧段，半径与轴线都不稳定 → 直接用已保存的圆柱结果。"""
    s = sv.SVG(FW, FH)
    # 左：小弧段
    s.rect(34, 30, 620, 430, fill=sv.GRAY_L, stroke=sv.LINE, sw=1.4)
    s.text(60, 70, "同一段点云，半径对不上一个数", size=22, fill=sv.RED, bold=True)
    cx, cy, R = 344, 2600, 2400
    pts = []
    for k in range(81):
        th = (-4.6 + 9.2 * k / 80) * math.pi / 180
        pts.append((cx + R * math.sin(th), cy - R * math.cos(th)))
    s.path(sv.catmull_path(pts), stroke=sv.BLUE, sw=3.2)
    s.line(pts[0][0], pts[0][1], cx, cy - R + 240, stroke=sv.FAINT, sw=1.4, dash="9 8")
    s.line(pts[-1][0], pts[-1][1], cx, cy - R + 240, stroke=sv.FAINT, sw=1.4, dash="9 8")
    s.circle(cx, cy - R + 240, 6, fill=sv.RED)
    s.text(430, 176, "圆心远在画面之外", size=16, fill=sv.RED)
    s.text(60, 180, "扫描到的弧段（约 8°）", size=17, fill=sv.BLUE)
    s.text(60, 340, "同一段点云，半径从 1200 到 4400 mm 都能吻合；", size=18, fill=sv.INK)
    s.text(60, 372, "拟合出的轴线也随之摆动。", size=18, fill=sv.INK)
    s.text(60, 412, "因此基准不能现场算，必须用一次稳定的拟合结果。", size=17, fill=sv.SUB)
    # 右：做法
    s.rect(676, 30, 590, 430, fill=sv.WHITE, stroke=sv.LINE, sw=1.4)
    s.text(702, 70, "做法：基准不在本功能里产生", size=22, fill=sv.BLUE, bold=True)
    box = [("圆柱功能：对远件拟合一次", sv.BLUE_L, sv.BLUE_D),
           ("保存结果（轴线 + 设计半径）", sv.WHITE, sv.INK),
           ("本功能：选择并使用它", sv.GRAY_L, sv.INK)]
    for i, (t, fill, color) in enumerate(box):
        y = 128 + i * 86
        s.rect(702, y, 538, 62, fill=fill, stroke=sv.LINE, sw=1.3)
        s.text(722, y + 38, t, size=19, fill=color, bold=(i == 0))
        if i < 2:
            s.line(971, y + 62, 971, y + 86, stroke=sv.BLUE, sw=2.0, marker_end="arrow")
    s.text(702, 404, "本功能不做圆柱拟合。", size=19, fill=sv.RED, bold=True)
    s.save(os.path.join(SVD, "weld_f2_基准.svg"))


def fig_unwrap():
    """难点②：两件没有共同的尺子 → 同一参考圆柱展开成平面图。"""
    s = sv.SVG(FW, FH)
    s.rect(34, 30, 470, 430, fill=sv.WHITE, stroke=sv.LINE, sw=1.4)
    s.text(60, 70, "两份点云，各自的坐标系", size=21, fill=sv.INK, bold=True)
    for i, (col, name) in enumerate(((sv.RED, "近件"), (sv.BLUE, "远件"))):
        y = 130 + i * 140
        s.rect(80, y, 300, 92, fill=sv.WHITE, stroke=col, sw=2.4)
        s.text(230, y + 54, name, size=20, fill=col, bold=True, anchor="middle")
        s.line(410, y + 46, 460, y + 46, stroke=col, sw=2.4)
        s.text(466, y + 36, "原点 / 角度零位", size=14, fill=sv.SUB)
        s.text(466, y + 58, "各自一套", size=14, fill=sv.SUB)
    s.text(60, 424, "直接相减会把坐标差当成缺陷。", size=17, fill=sv.RED)
    s.line(516, 245, 566, 245, stroke=sv.BLUE, sw=2.6, marker_end="arrow")
    s.text(541, 226, "同一参考圆柱", size=15, fill=sv.BLUE, anchor="middle")
    # 右：展开平面图
    s.rect(580, 30, 686, 430, fill=sv.GRAY_L, stroke=sv.LINE, sw=1.4)
    s.text(606, 70, "展开成平面图后比较", size=21, fill=sv.BLUE, bold=True)
    X0, Y0, X1, Y1 = 660, 118, 1230, 396
    s.rect(X0, Y0, X1 - X0, Y1 - Y0, fill=sv.WHITE, stroke=sv.LINE, sw=1.2)
    s.line(X0, Y1, X1, Y1, stroke=sv.INK, sw=1.8, marker_end="arrow")
    s.text(X1 - 6, Y1 + 26, "轴向 a", size=16, fill=sv.SUB, anchor="end")
    s.line(X0, Y1, X0, Y0, stroke=sv.INK, sw=1.8, marker_end="arrow")
    s.text(X0 - 8, Y0 + 8, "周向弧长 s", size=16, fill=sv.SUB, anchor="end")
    s.path(sv.catmull_path([(X0 + 20, Y0 + 150), (X0 + 180, Y0 + 120), (X0 + 340, Y0 + 138),
                            (X1 - 30, Y0 + 118)]), stroke=sv.RED, sw=2.6)
    s.text(X0 + 26, Y0 + 108, "近件缝边", size=16, fill=sv.RED, bold=True)
    s.path(sv.catmull_path([(X0 + 20, Y0 + 196), (X0 + 180, Y0 + 168), (X0 + 340, Y0 + 186),
                            (X1 - 30, Y0 + 164)]), stroke=sv.BLUE, sw=2.6)
    s.text(X0 + 26, Y0 + 214, "远件缝边", size=16, fill=sv.BLUE, bold=True)
    s.line(X0 + 250, Y0 + 96, X0 + 250, Y0 + 236, stroke=sv.FAINT, sw=1.6, dash="8 7")
    s.text(X0 + 258, Y0 + 252, "同一 s 上做比较", size=15, fill=sv.SUB)
    s.save(os.path.join(SVD, "weld_f3_展开.svg"))


def fig_edge():
    """难点③：缝边判据 + 外推到真实板边。"""
    s = sv.SVG(FW, FH)
    s.rect(30, 30, 392, 430, fill=sv.WHITE, stroke=sv.LINE, sw=1.4)
    s.text(56, 70, "判据：这边没料了", size=21, fill=sv.BLUE, bold=True)
    import math
    ox, oy, r = 226, 268, 96
    s.circle(ox, oy, 7, fill=sv.RED)
    s.text(ox, oy + 52, "该点", size=17, fill=sv.RED, anchor="middle", bold=True)
    for a in range(-90, 91, 18):
        th = math.radians(a)
        s.line(ox, oy, ox + r * math.sin(th), oy - r * math.cos(th), stroke=sv.BLUE_M, sw=1.4)
        s.circle(ox + r * math.sin(th), oy - r * math.cos(th), 3.4, fill=sv.BLUE)
    s.path(sv.catmull_path([(ox - r * 0.98, oy + 8), (ox, oy + r * 0.98), (ox + r * 0.98, oy + 8)]),
           stroke=sv.RED, sw=3.0)
    s.text(ox, oy + r + 54, "空缺方向", size=16, fill=sv.RED, anchor="middle")
    s.rect(56, 392, 340, 46, fill=sv.BLUE_L, stroke="none")
    s.text(70, 423, "一侧有料、一侧空着 → 支持率约一半", size=16, fill=sv.BLUE_D)
    # 中：朝向对面件
    s.rect(438, 30, 392, 430, fill=sv.GRAY_L, stroke=sv.LINE, sw=1.4)
    s.text(464, 70, "空缺必须朝向对面件", size=21, fill=sv.INK, bold=True)
    s.rect(500, 168, 268, 92, fill=sv.WHITE, stroke=sv.BLUE, sw=2.2)
    s.text(634, 222, "对面那一件", size=18, fill=sv.BLUE, anchor="middle")
    s.line(634, 120, 634, 160, stroke=sv.FAINT, sw=1.6, dash="7 6")
    s.text(634, 108, "外轮廓", size=15, fill=sv.FAINT, anchor="middle")
    s.line(470, 214, 494, 236, stroke=sv.FAINT, sw=1.6, dash="7 6")
    s.text(458, 208, "孔洞", size=15, fill=sv.FAINT, anchor="end")
    s.line(790, 214, 770, 236, stroke=sv.FAINT, sw=1.6, dash="7 6")
    s.text(822, 208, "裁剪边界", size=15, fill=sv.FAINT, anchor="end")
    s.text(464, 386, "朝向别处的空缺不是缝边，直接排除。", size=17, fill=sv.SUB)
    # 右：外推（讲清"点停在材料内侧 -> 推到真实边界"）
    s.rect(846, 30, 420, 430, fill=sv.WHITE, stroke=sv.LINE, sw=1.4)
    s.text(872, 70, "外推到材料真实边界", size=21, fill=sv.BLUE, bold=True)
    bx = 1092                                     # 材料边界位置
    s.rect(880, 152, bx - 880, 176, fill=sv.GRAY_L, stroke="none")
    s.text(890, 176, "有料一侧", size=15, fill=sv.SUB)
    s.text(bx + 12, 300, "空缺（缝）", size=15, fill=sv.FAINT)
    s.line(bx, 132, bx, 350, stroke=sv.INK, sw=2.6)
    s.text(bx + 12, 152, "材料边界", size=15, fill=sv.INK)
    px = bx - 104
    s.circle(px, 238, 7, fill=sv.RED)
    s.text(px - 6, 214, "检测到的边缘点", size=15, fill=sv.RED, anchor="middle")
    s.text(px - 6, 268, "（在材料内侧）", size=14, fill=sv.SUB, anchor="middle")
    s.line(px + 12, 238, bx - 6, 238, stroke=sv.RED, sw=2.4, marker_end="arrow")
    s.text((px + bx) / 2, 232, "d", size=17, fill=sv.RED, anchor="middle", bold=True)
    s.circle(bx, 238, 5, fill=sv.WHITE, stroke=sv.RED, sw=2.0)
    s.text(872, 386, "点停在材料内侧，间隙就会被算大；", size=16, fill=sv.SUB)
    s.text(872, 416, "推到真实边界后，量到的才是开口。", size=16, fill=sv.BLUE)
    s.text(872, 444, "d 由邻域半径与空缺角算出。", size=14, fill=sv.FAINT)
    s.save(os.path.join(SVD, "weld_f4_缝边.svg"))


def fig_pair():
    """难点④：沿轴向求交配对 + 拒绝情形。"""
    s = sv.SVG(FW, FH)
    s.rect(30, 30, 800, 430, fill=sv.WHITE, stroke=sv.LINE, sw=1.4)
    s.text(56, 70, "沿轴向求交，不找最近点", size=21, fill=sv.BLUE, bold=True)
    X0, Y0, X1, Y1 = 110, 122, 800, 366
    s.rect(X0, Y0, X1 - X0, Y1 - Y0, fill=sv.GRAY_L, stroke=sv.LINE, sw=1.2)
    s.line(X0, Y1, X1, Y1, stroke=sv.INK, sw=1.8, marker_end="arrow")
    s.text(X1 - 8, Y1 + 26, "轴向 a", size=16, fill=sv.SUB, anchor="end")
    s.line(X0, Y1, X0, Y0, stroke=sv.INK, sw=1.8, marker_end="arrow")
    s.text(X0 - 8, Y0 + 10, "周向 s", size=16, fill=sv.SUB, anchor="end")
    s.line(X0 + 40, Y0 + 96, X1 - 40, Y0 + 96, stroke=sv.BLUE, sw=2.6)
    s.text(X0 + 46, Y0 + 86, "远件缝边曲线", size=16, fill=sv.BLUE)
    s.line(X0 + 40, Y0 + 176, X1 - 40, Y0 + 176, stroke=sv.RED, sw=2.6)
    s.text(X0 + 46, Y0 + 200, "近件缝边", size=16, fill=sv.RED)
    px, py = X0 + 190, Y0 + 176
    s.circle(px, py, 7, fill=sv.RED)
    s.line(px, py, px, Y0 + 96, stroke=sv.RED, sw=2.4, marker_end="arrow")
    s.circle(px, Y0 + 96, 6, fill=sv.BLUE)
    s.text(px + 14, Y0 + 76, "对应点", size=16, fill=sv.BLUE)
    s.text(px + 14, py - 12, "近件缝边点", size=16, fill=sv.RED)
    s.text(X0 + 372, Y0 + 148, "同一周向位置上量距离", size=17, fill=sv.SUB)
    s.text(X0 + 372, Y0 + 210, "配对方向固定为轴向，", size=17, fill=sv.BLUE)
    s.text(X0 + 372, Y0 + 242, "因此对应点是唯一的。", size=17, fill=sv.BLUE)
    # 右：三种情形
    s.rect(846, 30, 420, 430, fill=sv.GRAY_L, stroke=sv.LINE, sw=1.4)
    s.text(872, 70, "配不上的情形", size=21, fill=sv.INK, bold=True)
    rows = [("范围里找不到交点", "放大范围再找一次", sv.BLUE),
            ("出现多个交点", "不给数：不可测", sv.RED),
            ("方向几乎平行", "不给数：不可测", sv.RED)]
    for i, (t, act, col) in enumerate(rows):
        y = 124 + i * 108
        s.rect(872, y, 368, 84, fill=sv.WHITE, stroke=sv.LINE, sw=1.3)
        s.rect(872, y, 5, 84, fill=col)
        s.text(896, y + 34, t, size=18, fill=sv.INK)
        s.text(896, y + 64, act, size=16, fill=col, bold=True)
    s.save(os.path.join(SVD, "weld_f5_配对.svg"))


def fig_two():
    """一个对应关系 → 两个量（几何关系与软件里一致：对应点沿轴向错开一个间隙）。"""
    s = sv.SVG(FW, FH)
    X0, Y0, X1, Y1 = 110, 96, 790, 400
    s.rect(X0, Y0, X1 - X0, Y1 - Y0, fill=sv.GRAY_L, stroke=sv.LINE, sw=1.2)
    s.line(X0, Y1, X1, Y1, stroke=sv.INK, sw=1.8, marker_end="arrow")
    s.text(X1 - 8, Y1 + 28, "轴向 a", size=16, fill=sv.SUB, anchor="end")
    s.line(X0, Y1, X0, Y0, stroke=sv.INK, sw=1.8, marker_end="arrow")
    s.text(X0 - 8, Y0 + 10, "径向 e", size=16, fill=sv.SUB, anchor="end")

    y_far, y_near = Y0 + 118, Y0 + 238
    x_near, x_far = X0 + 250, X0 + 420
    # 近件材料面 / 远件材料面（各自在缝边处断开）
    s.line(X0 + 30, y_near, x_near, y_near, stroke=sv.RED, sw=3.0)
    s.text(X0 + 36, y_near + 30, "近件材料面", size=16, fill=sv.RED)
    s.line(x_far, y_far, X1 - 30, y_far, stroke=sv.BLUE, sw=3.0)
    s.text(X1 - 36, y_far - 14, "远件材料面", size=16, fill=sv.BLUE, anchor="end")
    s.circle(x_near, y_near, 7, fill=sv.RED)
    s.circle(x_far, y_far, 7, fill=sv.BLUE)
    s.text(x_near - 14, y_near + 6, "P", size=19, fill=sv.RED, anchor="end", bold=True)
    s.text(x_far + 16, y_far + 6, "Q", size=19, fill=sv.BLUE, bold=True)
    # 对应关系连线（P→Q）
    s.line(x_near, y_near, x_far, y_far, stroke=sv.SUB, sw=2.4)
    s.text(x_far + 34, y_far - 26, "对应关系连线", size=15, fill=sv.SUB)
    # 阶差：同一站位上的径向距离
    xm = x_near - 66
    s.line(xm, y_far, xm, y_near, stroke=sv.BLUE, sw=3.2, marker_end="arrow")
    s.text(xm - 12, (y_far + y_near) / 2 + 6, "阶差（径向）", size=16, fill=sv.BLUE, anchor="end")
    s.line(x_near, y_far, x_far, y_far, stroke=sv.BLUE, sw=1.2, dash="6 6")
    s.line(xm, y_far, x_near, y_far, stroke=sv.FAINT, sw=1.2, dash="6 6")
    # 间隙：两个缝边之间沿轴向的距离
    yg = y_near + 42
    s.line(x_near, yg, x_far, yg, stroke=sv.RED, sw=3.2, marker_end="arrow")
    s.line(x_near, y_near, x_near, yg + 7, stroke=sv.FAINT, sw=1.1, dash="5 5")
    s.line(x_far, y_far, x_far, yg + 7, stroke=sv.FAINT, sw=1.1, dash="5 5")
    s.text(x_far + 16, yg + 6, "间隙（沿轴向）", size=16, fill=sv.RED)

    # 右侧说明
    s.rect(830, 96, 436, 300, fill=sv.WHITE, stroke=sv.LINE, sw=1.4)
    s.text(858, 148, "阶差 = e_P − e_Q", size=25, fill=sv.BLUE, bold=True)
    s.text(858, 184, "正值：近件在外", size=17, fill=sv.BLUE_D)
    s.text(858, 250, "间隙 = 沿轴向的距离", size=25, fill=sv.RED, bold=True)
    s.text(858, 286, "两条缝边之间的开口", size=17, fill=sv.SUB)
    s.rect(830, 412, 436, 48, fill=sv.BLUE_L, stroke="none")
    s.text(858, 443, "一次配对，两个量同时得到。", size=19, fill=sv.BLUE_D)
    s.save(os.path.join(SVD, "weld_f6_两量.svg"))


def fig_flow():
    """操作流程：六步横条（用于操作流程页顶部）。"""
    W, H = 1300, 190
    s = sv.SVG(W, H)
    steps = ["对远件拟合圆柱并保存", "选择圆柱结果", "选择远件",
             "选择近件", "开始计算", "查看结果与标注"]
    n = len(steps)
    gap, x0 = 20, 24
    bw = (W - 2 * x0 - (n - 1) * gap) / n
    for i, t in enumerate(steps):
        x = x0 + i * (bw + gap)
        s.rect(x, 42, bw, 104, fill=sv.WHITE, stroke=sv.LINE, sw=1.4)
        s.rect(x, 42, bw, 4, fill=sv.BLUE)
        sv.step_badge(s, x + 24, 74, str(i + 1))
        s.text(x + 46, 80, t, size=17, fill=sv.INK, bold=True)
        if i < n - 1:
            s.line(x + bw + 2, 94, x + bw + gap - 2, 94, stroke=sv.BLUE, sw=2.2, marker_end="arrow")
    s.save(os.path.join(SVD, "weld_f7_流程.svg"))


# ============================================================ 版式辅助（全部直角）
def lead_flat(s, text):
    dk.rect(s, 0.5, 1.18, 12.33, 0.94, fill=dk.GRAY_L, shape=MSO_SHAPE.RECTANGLE)
    dk.rect(s, 0.5, 1.24, 0.06, 0.82, fill=dk.BLUE, shape=MSO_SHAPE.RECTANGLE)
    dk.textbox(s, 0.78, 1.28, 11.9, 0.78, text, size=15, color=dk.INK, line_spacing=1.32)


def para(s, y, lines, size=15):
    dk.textbox(s, 0.5, y, 12.33, 0.9, lines, size=size, color=dk.SUB, line_spacing=1.3)


def ph(s, x, y, w, h, label, hint=None):
    sh = dk.rect(s, x, y, w, h, fill=dk.GRAY_L, line=dk.FAINT, lw=1.5,
                 dash=MSO_LINE_DASH_STYLE.DASH, shape=MSO_SHAPE.RECTANGLE)
    tf = sh.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    p = tf.paragraphs[0]; p.alignment = PP_ALIGN.CENTER
    r = p.add_run(); r.text = "【占位图】" + label
    dk.set_font(r, 16, True, dk.SUB)
    if hint:
        p2 = tf.add_paragraph(); p2.alignment = PP_ALIGN.CENTER
        r2 = p2.add_run(); r2.text = hint
        dk.set_font(r2, 13, False, dk.FAINT)


def fig(s, name, y=2.30, h=4.68, x=0.5, w=12.33):
    return dk.picture_fit(s, os.path.join(PNG, name), x, y, w, h)


# ============================================================ 组页
def build():
    for f in (fig_link, fig_base, fig_unwrap, fig_edge, fig_pair, fig_two, fig_flow):
        f()
    for f in ("weld_f1_链路", "weld_f2_基准", "weld_f3_展开", "weld_f4_缝边",
              "weld_f5_配对", "weld_f6_两量", "weld_f7_流程"):
        subprocess.run(["inkscape", os.path.join(SVD, f + ".svg"),
                        "-o", os.path.join(PNG, f + ".png"), "-w", "2400"], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    prs = Presentation()
    prs.slide_width, prs.slide_height = Inches(dk.SW), Inches(dk.SH)
    NOTE = "焊前装配测量 · 思路与流程"
    n = 0

    # ---- 1 封面
    n += 1; s = dk.new_slide(prs)
    dk.textbox(s, 0.9, 1.55, 11.6, 0.5, "CloudForge Analyzer · 点云测量功能", size=15, color=dk.SUB)
    dk.textbox(s, 0.9, 2.05, 11.6, 1.0, "焊前装配测量", size=44, bold=True, color=dk.INK)
    dk.textbox(s, 0.9, 3.05, 11.6, 0.6, "径向阶差与轴向间隙 —— 思路、难点与操作流程",
               size=22, color=dk.BLUE)
    dk.rect(s, 0.9, 4.05, 11.5, 0.018, fill=dk.LINE, shape=MSO_SHAPE.RECTANGLE)
    dk.footer(s, n, NOTE)

    # ---- 2 总体思路
    n += 1; s = dk.new_slide(prs)
    dk.title(s, "总体思路", "六步：选基准 → 选两件 → 展开 → 找缝边 → 配对 → 出结果")
    lead_flat(s, "两件点云先放进同一个参考圆柱里摊平成平面图，再沿轴向把两边的缝边一一对应；"
                 "一对对应点同时给出径向阶差与轴向间隙。")
    fig(s, "weld_f1_链路.png")
    dk.footer(s, n, NOTE)

    # ---- 3 难点①
    n += 1; s = dk.new_slide(prs)
    dk.title(s, "难点一：基准圆柱不能现算", "大半径、小弧段")
    lead_flat(s, "零件半径很大（约 1940 mm），一次扫描只覆盖很小一段圆弧，用点云反算半径和轴线都不稳定。")
    para(s, 2.24, "做法：基准由圆柱功能对远件拟合一次并保存；本功能只选择并使用这条轴线与设计半径，不再拟合。")
    fig(s, "weld_f2_基准.png", y=2.78, h=4.2)
    dk.footer(s, n, NOTE)

    # ---- 4 难点②
    n += 1; s = dk.new_slide(prs)
    dk.title(s, "难点二：两件没有共同的尺子", "各自的坐标系不能直接相减")
    lead_flat(s, "远件与近件是两份独立的点云，原点与角度零位各不相同，直接比较会把坐标差当成缺陷。")
    para(s, 2.24, "做法：两件都投影到同一个参考圆柱，展开成“轴向 × 周向弧长”的平面图，所有比较都在这张图上进行。")
    fig(s, "weld_f3_展开.png", y=2.78, h=4.2)
    dk.footer(s, n, NOTE)

    # ---- 5 难点③
    n += 1; s = dk.new_slide(prs)
    dk.title(s, "难点三：缝边怎么自动找", "不靠人工标注")
    lead_flat(s, "逐点判断“我这边没料了，而且空缺的方向正对着对面那一件”，满足这条判据的点就是缝边。")
    para(s, 2.24, "再把边缘点沿空缺方向推到材料真实边界上，避免边缘点落在材料内侧、把间隙算大。")
    fig(s, "weld_f4_缝边.png", y=2.78, h=4.2)
    dk.footer(s, n, NOTE)

    # ---- 6 难点④
    n += 1; s = dk.new_slide(prs)
    dk.title(s, "难点四：两边的点怎么对应", "沿轴向求交，不是找最近点")
    lead_flat(s, "从每个近件缝边点沿轴向射一条线，撞到远件缝边曲线的位置才是它的对应点；"
                 "同一条线上量出来的距离才有物理意义。")
    para(s, 2.24, "找不到交点时自动放大搜索范围再找一次；出现多个交点或方向几乎平行时不给数，如实报为不可测。")
    fig(s, "weld_f5_配对.png", y=2.78, h=4.2)
    dk.footer(s, n, NOTE)

    # ---- 7 一个对应关系 → 两个量
    n += 1; s = dk.new_slide(prs)
    dk.title(s, "一个对应关系，两个量", "阶差与间隙同源")
    lead_flat(s, "同一对对应点里，径向的高度差就是阶差，沿轴向的距离就是间隙——两个量互不干扰，一次配对即可同时得到。")
    fig(s, "weld_f6_两量.png", y=2.30, h=4.68)
    dk.footer(s, n, NOTE)

    # ---- 8 操作流程（含两个截图占位）
    n += 1; s = dk.new_slide(prs)
    dk.title(s, "操作流程", "无需填写任何参数")
    lead_flat(s, "先用圆柱功能对远件拟合一次并保存结果；之后每次测量只需选择基准与两件点云。")
    dk.picture_fit(s, os.path.join(PNG, "weld_f7_流程.png"), 0.5, 2.26, 12.33, 1.28, border=False)
    ph(s, 0.5, 3.70, 6.05, 3.05, "圆柱结果选择窗口", "从已保存的圆柱结果中选择本次基准")
    ph(s, 6.78, 3.70, 6.05, 3.05, "远件 / 近件点云选择窗口", "依次选择两份点云")
    dk.footer(s, n, NOTE)

    # ---- 9 总体效果
    n += 1; s = dk.new_slide(prs)
    dk.title(s, "总体效果", "一次测量，两个量 + 三维标注")
    lead_flat(s, "测量完成后，三维窗口中给出两件缝边、点对点对应关系，以及阶差与间隙最大值的位置与数值。")
    ph(s, 0.5, 2.30, 12.33, 4.45, "总体效果图",
       "三维窗口：两条缝边 + 稀疏对应线 + 最大值处的标注")
    dk.footer(s, n, NOTE)

    # ---- 10 可视化细节
    n += 1; s = dk.new_slide(prs)
    dk.title(s, "可视化细节", "左侧看配对，右侧看最大值")
    lead_flat(s, "左图用来确认两边的点是否一一对应正确；右图给出最大值出现在哪里、两个量各是多少。")
    ph(s, 0.5, 2.30, 6.05, 4.30, "缝边与对应关系", "绿色/黄色缝边 + 稀疏青色对应线")
    ph(s, 6.78, 2.30, 6.05, 4.30, "最大值处标注", "红色弧线=间隙；蓝色线=阶差；橙色线=两点连线")
    dk.footer(s, n, NOTE)

    # ---- 11 计算结果
    n += 1; s = dk.new_slide(prs)
    dk.title(s, "计算结果", "整体中值 + 最大值 + 位置")
    lead_flat(s, "结果给出每个量的整体中值与最大值，并给出最大值的三维位置与展开位置；"
                 "有缺测段时如实列出，不外推。")
    ph(s, 0.5, 2.30, 12.33, 3.60, "计算结果截图", "报告区文本结果")
    para(s, 6.10, "结论：一次操作同时得到阶差与间隙两个量及其所在位置。", size=17)
    dk.footer(s, n, NOTE)

    prs.save(OUT)
    print("pages =", len(prs.slides.__iter__.__self__._sldIdLst))
    print("saved:", OUT, os.path.getsize(OUT) // 1024, "KB")


if __name__ == "__main__":
    build()
