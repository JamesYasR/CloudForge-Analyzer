# -*- coding: utf-8 -*-
"""极简 SVG 生成工具：为凹塘测量 PPT 绘制原理图。

设计约束
  * 只用 Inkscape/浏览器都支持的最基础特性（rect/line/path/text/marker/pattern），
    避免 filter/gradient 依赖，保证渲染一致；
  * 中文字体固定 Noto Sans CJK SC（本机已装）；
  * 文本宽度用近似算法估算，便于自动检查越界与重叠（见 check_svg.py）。
"""
import math

# ---------------------------------------------------------------- 配色
# 只用两套色：BLUE=数据/基准/结果/说明（正常），RED=缺陷/问题/局限（需要注意），
# 其余一律灰阶。旧常量名保留为别名，避免各脚本大改。
INK = "#1B2733"        # 主文字
SUB = "#5A6672"        # 次级文字
FAINT = "#98A4B0"      # 更弱
LINE = "#CBD5DE"       # 分隔线 / 网格
GRAY_L = "#F4F7FA"     # 面板底
WHITE = "#FFFFFF"

BLUE = "#1F5C99"       # 主色
BLUE_L = "#E3EDF7"     # 主色浅底
BLUE_M = "#7FA8CC"     # 主色中间调（拟合线等）
BLUE_D = "#16446F"     # 主色深（浅底上的文字）

RED = "#C0392B"        # 需要注意
RED_L = "#FBE7E4"      # 需要注意·浅底

# --- 兼容别名（旧脚本仍在用这些名字）---
ORANGE, ORANGE_L = BLUE, BLUE_L
PURPLE, PURPLE_L = BLUE, BLUE_L
GREEN, GREEN_L = BLUE, BLUE_L
YELLOW_L, YELLOW_D = BLUE_L, BLUE
MAGENTA = RED

FONT = "Noto Sans CJK SC"

# 近似字宽（em）：CJK=1.0，大写/数字=0.60，小写=0.52，空格=0.28
def text_width(s, size, bold=False):
    w = 0.0
    for ch in s:
        o = ord(ch)
        if o > 0x2E80:
            w += 1.0
        elif ch in " ":
            w += 0.30
        elif ch.isdigit():
            w += 0.58
        elif ch.isupper():
            w += 0.66
        elif ch in ".,:;'|!":
            w += 0.30
        elif ch in "-–—+/()[]<>=":
            w += 0.45
        else:
            w += 0.55
    if bold:
        w *= 1.06
    return w * size


class SVG:
    def __init__(self, w, h, bg=WHITE):
        self.w, self.h = w, h
        self.bg = bg
        self.body = []
        self.defs = []
        self._mid = 0
        self.texts = []          # (x, y, width, height, string) 用于自动检查

    # ---------- defs ----------
    def marker_arrow(self, name, color, size=12):
        self.defs.append(
            f'<marker id="{name}" viewBox="0 0 12 12" refX="9.5" refY="6" '
            f'markerWidth="{size}" markerHeight="{size}" orient="auto-start-reverse">'
            f'<path d="M1,1 L11,6 L1,11 z" fill="{color}"/></marker>')
        return f"url(#{name})"

    def marker_dot(self, name, color, r=4.5):
        self.defs.append(
            f'<marker id="{name}" viewBox="0 0 12 12" refX="6" refY="6" '
            f'markerWidth="{r}" markerHeight="{r}">'
            f'<circle cx="6" cy="6" r="5.4" fill="{color}" stroke="{WHITE}" stroke-width="1.6"/></marker>')
        return f"url(#{name})"

    def pattern_hatch(self, name, color, step=9, sw=1.1, bg="none", angle=45):
        self.defs.append(
            f'<pattern id="{name}" width="{step}" height="{step}" patternUnits="userSpaceOnUse" '
            f'patternTransform="rotate({angle})">'
            f'<rect width="{step}" height="{step}" fill="{bg}"/>'
            f'<line x1="0" y1="0" x2="0" y2="{step}" stroke="{color}" stroke-width="{sw}"/></pattern>')
        return f"url(#{name})"

    def pattern_dots(self, name, color, step=11, r=1.5):
        self.defs.append(
            f'<pattern id="{name}" width="{step}" height="{step}" patternUnits="userSpaceOnUse">'
            f'<circle cx="{step/2}" cy="{step/2}" r="{r}" fill="{color}"/></pattern>')
        return f"url(#{name})"

    def clip(self, name, shape):
        self.defs.append(f'<clipPath id="{name}">{shape}</clipPath>')
        return f"url(#{name})"

    # ---------- 基础图元 ----------
    def rect(self, x, y, w, h, fill="none", stroke="none", sw=1, rx=0, dash=None, op=None, clip=None):
        s = f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" rx="{rx}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"'
        if dash:
            s += f' stroke-dasharray="{dash}"'
        if op is not None:
            s += f' opacity="{op}"'
        if clip:
            s += f' clip-path="{clip}"'
        self.body.append(s + "/>")

    def panel(self, x, y, w, h, fill=GRAY_L, stroke=LINE, sw=1.4, rx=14, dash=None, op=None, clip=None):
        self.rect(x, y, w, h, fill=fill, stroke=stroke, sw=sw, rx=rx, dash=dash, op=op, clip=clip)

    def line(self, x1, y1, x2, y2, stroke=INK, sw=1.6, dash=None, marker_end=None,
             marker_start=None, cap="round", op=None):
        s = (f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
             f'stroke="{stroke}" stroke-width="{sw}" stroke-linecap="{cap}"')
        if dash:
            s += f' stroke-dasharray="{dash}"'
        if marker_end:
            s += f' marker-end="{marker_end}"'
        if marker_start:
            s += f' marker-start="{marker_start}"'
        if op is not None:
            s += f' opacity="{op}"'
        self.body.append(s + "/>")

    def circle(self, cx, cy, r, fill="none", stroke="none", sw=1.4, dash=None, op=None):
        s = f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="{r:.2f}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"'
        if dash:
            s += f' stroke-dasharray="{dash}"'
        if op is not None:
            s += f' opacity="{op}"'
        self.body.append(s + "/>")

    def ellipse(self, cx, cy, rx, ry, fill="none", stroke="none", sw=1.4, dash=None,
                op=None, rot=0):
        s = (f'<ellipse cx="{cx:.2f}" cy="{cy:.2f}" rx="{rx:.2f}" ry="{ry:.2f}" '
             f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}"')
        if rot:
            s += f' transform="rotate({rot:.2f} {cx:.2f} {cy:.2f})"'
        if dash:
            s += f' stroke-dasharray="{dash}"'
        if op is not None:
            s += f' opacity="{op}"'
        self.body.append(s + "/>")

    def path(self, d, fill="none", stroke="none", sw=1.8, dash=None, marker_end=None,
             marker_start=None, cap="round", join="round", op=None, clip=None):
        s = (f'<path d="{d}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" '
             f'stroke-linecap="{cap}" stroke-linejoin="{join}"')
        if dash:
            s += f' stroke-dasharray="{dash}"'
        if marker_end:
            s += f' marker-end="{marker_end}"'
        if marker_start:
            s += f' marker-start="{marker_start}"'
        if op is not None:
            s += f' opacity="{op}"'
        if clip:
            s += f' clip-path="{clip}"'
        self.body.append(s + "/>")

    def poly(self, pts, fill="none", stroke="none", sw=1.8, dash=None, op=None, close=True):
        d = "M" + " L".join(f"{x:.2f},{y:.2f}" for x, y in pts) + (" Z" if close else "")
        self.path(d, fill=fill, stroke=stroke, sw=sw, dash=dash, op=op)

    # ---------- 文本 ----------
    def text(self, x, y, s, size=22, fill=INK, anchor="start", bold=False, italic=False,
             family=FONT, lh=None, track=False, op=None):
        """y 为基线位置。多行用 \n。返回文本块高度（近似）。"""
        lines = s.split("\n")
        lh = lh or size * 1.35
        for i, ln in enumerate(lines):
            yy = y + i * lh
            st = (f'<text x="{x:.2f}" y="{yy:.2f}" font-family="{family}" font-size="{size}" '
                  f'fill="{fill}" text-anchor="{anchor}"')
            if bold:
                st += ' font-weight="700"'
            if italic:
                st += ' font-style="italic"'
            if track:
                st += f' letter-spacing="{size*0.06:.2f}"'
            if op is not None:
                st += f' opacity="{op}"'
            st += f'>{esc(ln)}</text>'
            self.body.append(st)
            wdt = text_width(ln, size, bold)
            x0 = x if anchor == "start" else (x - wdt if anchor == "end" else x - wdt / 2)
            self.texts.append((x0, yy - size * 0.82, wdt, size * 1.02, ln))
        return lh * len(lines)

    def label_box(self, x, y, s, size=20, pad=8, fill=WHITE, stroke=LINE, sw=1.2,
                  color=INK, rx=8, bold=False, anchor="middle"):
        """带圆角的文字标签（x,y 为中心）。返回 (w,h)。"""
        tw = max(text_width(l, size, bold) for l in s.split("\n"))
        th = size * 1.35 * len(s.split("\n"))
        w, h = tw + pad * 2, th + pad * 1.6
        self.rect(x - w / 2, y - h / 2, w, h, fill=fill, stroke=stroke, sw=sw, rx=rx)
        self.text(x, y + size * 0.36 - (th - size * 1.35) / 2, s, size=size, fill=color,
                  anchor="middle", bold=bold)
        return w, h

    # ---------- 输出 ----------
    def strip_title(self, crop, n=3):
        """去掉顶部标题带（title_bar 产生的前 n 个元素）并整体上移 crop 像素。"""
        self.body = self.body[n:]
        self.crop = crop
        return self

    def render(self):
        crop = getattr(self, "crop", 0)
        h = self.h - crop
        defs = f'<defs>{"".join(self.defs)}</defs>' if self.defs else ""
        body = "".join(self.body)
        if crop:
            body = f'<g transform="translate(0,{-crop})">{body}</g>'
        return (f'<?xml version="1.0" encoding="UTF-8"?>\n'
                f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{h}" '
                f'viewBox="0 0 {self.w} {h}">\n'
                f'<rect width="{self.w}" height="{h}" fill="{self.bg}"/>\n'
                f'{defs}\n{body}\n</svg>\n')

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            f.write(self.render())
        return path

    # ---------- 校验 ----------
    def overflow(self, margin=0.0):
        """返回越界文本列表。"""
        bad = []
        for x, y, w, h, s in self.texts:
            if x < -margin or y < -margin or x + w > self.w + margin or y + h > self.h + margin:
                bad.append((s[:26], round(x), round(y), round(w), round(h)))
        return bad


def esc(s):
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


# ---------------------------------------------------------------- 常用组合件
def title_bar(sv, x, y, w, text, sub=None, size=30, color=BLUE, h=54):
    """左上角标题条：竖色块 + 标题（+副标题）。"""
    sv.rect(x, y + 4, 6, h - 8, fill=color, rx=3)
    sv.text(x + 18, y + size * 0.92, text, size=size, fill=INK, bold=True)
    if sub:
        sv.text(x + 18 + text_width(text, size, True) + 16, y + size * 0.92 + 2,
                sub, size=size * 0.62, fill=SUB)


def chip(sv, x, y, s, size=18, fill=BLUE_L, color=BLUE, pad=11, h=32, bold=False):
    """小圆角标签，(x,y) 为左上角。返回宽度。"""
    w = text_width(s, size, bold) + pad * 2
    sv.rect(x, y, w, h, fill=fill, stroke="none", rx=h / 2)
    sv.text(x + w / 2, y + h / 2 + size * 0.35, s, size=size, fill=color, anchor="middle", bold=bold)
    return w


def legend_row(sv, x, y, items, size=19, gap=26, swatch=(22, 14), rx=3):
    """一行图例。items=[(颜色, 文字[, "dash"])]；带 "dash" 的画虚线空心块。"""
    cx = x
    for it in items:
        col, lab = it[0], it[1]
        dash = it[2] if len(it) > 2 else None
        sw_, sh = swatch
        if dash:
            sv.rect(cx, y - sh + 3, sw_, sh, fill=WHITE, stroke=col, sw=1.6, rx=rx,
                    dash="6 4")
        else:
            sv.rect(cx, y - sh + 3, sw_, sh, fill=col, stroke="none", rx=rx)
        sv.text(cx + sw_ + 9, y, lab, size=size, fill=SUB)
        cx += sw_ + 9 + text_width(lab, size) + gap
    return cx


def step_badge(sv, cx, cy, n, r=19, fill=BLUE, color=WHITE, size=21):
    sv.circle(cx, cy, r, fill=fill)
    sv.text(cx, cy + size * 0.36, str(n), size=size, fill=color, anchor="middle", bold=True)


def curve_pts(fn, x0, x1, n=160):
    return [(x0 + (x1 - x0) * i / (n - 1), fn(x0 + (x1 - x0) * i / (n - 1))) for i in range(n)]


def catmull_path(pts, close=False):
    """平滑折线 → 三次贝塞尔路径（Catmull-Rom 转 Bezier）。"""
    if len(pts) < 3:
        return "M" + " L".join(f"{x:.2f},{y:.2f}" for x, y in pts)
    d = f"M{pts[0][0]:.2f},{pts[0][1]:.2f}"
    for i in range(len(pts) - 1):
        p0 = pts[i - 1] if i > 0 else pts[i]
        p1, p2 = pts[i], pts[i + 1]
        p3 = pts[i + 2] if i + 2 < len(pts) else p2
        c1 = (p1[0] + (p2[0] - p0[0]) / 6, p1[1] + (p2[1] - p0[1]) / 6)
        c2 = (p2[0] - (p3[0] - p1[0]) / 6, p2[1] - (p3[1] - p1[1]) / 6)
        d += (f" C{c1[0]:.2f},{c1[1]:.2f} {c2[0]:.2f},{c2[1]:.2f} "
              f"{p2[0]:.2f},{p2[1]:.2f}")
    if close:
        d += " Z"
    return d


def area_path(fn, x0, x1, base, n=160, up=True):
    """函数曲线与基准线围成的填充区域路径。"""
    pts = curve_pts(fn, x0, x1, n)
    d = f"M{pts[0][0]:.2f},{base:.2f} L" + " L".join(f"{x:.2f},{y:.2f}" for x, y in pts)
    d += f" L{pts[-1][0]:.2f},{base:.2f} Z"
    return d


def wrap_arrow(sv, x, y, text, size=18, color=SUB, width=260):
    """自动换行的说明文字（按估算宽度折行）。"""
    out, cur = [], ""
    for ch in text:
        if text_width(cur + ch, size) > width:
            out.append(cur)
            cur = ch
        else:
            cur += ch
    out.append(cur)
    sv.text(x, y, "\n".join(out), size=size, fill=color, lh=size * 1.5)
    return len(out)


def dent_profile(t, depth, half, flat=0.18):
    """归一化凹坑剖面：t∈[-1,1] → 0..depth（余弦碗，边缘平滑）。"""
    a = abs(t)
    if a >= 1:
        return 0.0
    # 平滑余弦碗：内部平底占比 flat
    if a < flat:
        return depth
    u = (a - flat) / (1 - flat)
    return depth * (1 + math.cos(math.pi * u)) / 2
