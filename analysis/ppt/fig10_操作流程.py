# -*- coding: utf-8 -*-
"""图10：操作流程（6 步）"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from svgkit import *

W, H = 1600, 900
sv = SVG(W, H)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../docs/当前需新增功能/ppt_assets/svg")
steps = [
    ("打开点云", "菜单 / 工具栏打开点云文件", "显示点数与坐标范围"),
    ("拟合圆柱", "“二次优化圆柱”拟合\n半径猜测/设计半径填 1940", "保存圆柱结果（半径、轴线）"),
    ("凹塘测量", "测量菜单 → 凹塘/凹坑测量\n填 9 个参数（默认即可）", "默认：窗口 90 mm、阈值 0.35 mm"),
    ("后台计算", "点“确定”后自动后台计算\n进度条 + 状态信息实时刷新", "界面不卡；可随时取消"),
    ("看 3D 结果", "热力图 / 逐坑球标记\n逐坑椭圆 / 切换显示", "主坑洋红、其余橙、未通过灰白"),
    ("读报告", "报告逐坑给出：深度、\n椭圆长短轴、校验结论", "报告里含本次使用参数"),
]
CW, CH, X0, Y0 = 232, 300, 40, 200
MG = (1600 - 40 - 6 * CW) / 5.0
for i, (t, do, out) in enumerate(steps):
    x = X0 + i * (CW + MG)
    sv.rect(x, Y0, CW, CH, fill="#FBFCFE", stroke=LINE, sw=1.4, rx=12)
    sv.rect(x, Y0, CW, 6, fill=BLUE, rx=3)
    step_badge(sv, x + 32, Y0 + 46, i + 1, r=18)
    sv.text(x + 60, Y0 + 53, t, size=21, fill=INK, bold=True)
    sv.text(x + 22, Y0 + 108, do, size=17, fill=INK, lh=26)
    sv.text(x + 22, Y0 + 218, "→ " + out, size=16, fill=SUB, lh=24)
    if i < 5:
        ax = x + CW + 3
        sv.path("M%d,%d L%d,%d" % (ax, Y0 + CH / 2, ax + MG - 7, Y0 + CH / 2),
                stroke=SUB, sw=2.6, marker_end=sv.marker_arrow("arz", SUB, 12))

sv.panel(40, 528, 1520, 186, fill=BLUE_L, stroke=BLUE)
sv.text(70, 566, "关键点 1：凹塘测量直接用“二次优化圆柱”保存下来的那条圆柱结果 —— "
                 "轴线点、轴线方向、设计半径三项原样沿用，凹塘内部不会再拟合一次。",
        size=19, fill=BLUE_D, bold=True)
sv.text(70, 600, "　　　　　 所以拟合时务必把设计半径（1940 mm）填进“半径猜测 / 设计半径”："
                 "圆柱度与凹塘用的是同一条轴线、同一个半径。", size=18, fill=BLUE_D)
sv.text(70, 638, "关键点 2：设计半径是已知条件，必须填对，否则小弧段上的自由拟合会跑出荒谬半径。",
        size=19, fill=BLUE_D)
sv.text(70, 674, "关键点 3：参数不确定时先用默认值（窗口 90 mm / 阈值 0.35 mm）跑一遍，"
                 "报告里会打印本次使用的参数。", size=19, fill=BLUE_D)

sv.text(40, 762, "验证样本（随软件提供的测试数据）：", size=20, fill=INK, bold=True)
sv.text(40, 800, "· 样本 A —— 含 2~3 mm 形面偏差 + 4 个已知凹坑（φ11~φ30 mm，深 0.5~1.5 mm）："
                 "期望检出 4 个、无多余检出", size=18, fill=SUB)
sv.text(40, 832, "· 样本 B —— 理想柱面上的单个凹坑：期望检出 1 个；"
                 "· 样本 C —— 真实扫描件（无真实凹坑）：期望不判定", size=18, fill=SUB)

bad = sv.overflow(2)
print("OVERFLOW:", bad) if bad else print("fig10 no overflow")
sv.strip_title(182, 0)
sv.save(os.path.join(OUT, "fig10_操作流程.svg"))
print("fig10 ok")
