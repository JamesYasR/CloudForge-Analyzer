# -*- coding: utf-8 -*-
"""生成焊前装配功能的图标：纯黑 + 透明背景，矢量绘制（与 icon_Pothole/icon_weldheight 同风格）。

表达内容：两块母材对接（左高右低 = 径向阶差），中间留缝；用两个小双箭头分别标出
          阶差（竖直）与间隙（水平）。
用法：python3 analysis/icons/make_weld_icon.py
"""
import os, subprocess

W = H = 64
SVG = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">
  <g fill="#000000" stroke="none">
    <rect x="4" y="19" width="24" height="9"/>
    <rect x="36" y="31" width="24" height="9"/>
  </g>
  <g stroke="#000000" stroke-width="2.4" stroke-linecap="round" fill="none">
    <!-- 阶差：竖直双箭头（两侧材料面之间的高度差） -->
    <path d="M32 21 L32 41"/>
    <path d="M28.2 24.5 L32 19.6 L35.8 24.5" fill="#000000" stroke="none"/>
    <path d="M28.2 37.5 L32 42.4 L35.8 37.5" fill="#000000" stroke="none"/>
    <!-- 间隙：水平双箭头（两侧端面的开口） -->
    <path d="M28 50 L36 50"/>
    <path d="M28.9 46.6 L23.8 50 L28.9 53.4" fill="#000000" stroke="none"/>
    <path d="M35.1 46.6 L40.2 50 L35.1 53.4" fill="#000000" stroke="none"/>
  </g>
</svg>
'''

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "../.."))
svg_path = os.path.join(HERE, "weld_prep_icon.svg")
png_path = os.path.join(ROOT, "Icons/icon_WeldStep.png")
open(svg_path, "w", encoding="utf-8").write(SVG)
subprocess.run(["inkscape", svg_path, "-o", png_path, "-w", str(W), "-h", str(H)], check=True)
print("icon ->", png_path)
