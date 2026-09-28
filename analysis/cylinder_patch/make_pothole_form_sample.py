#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_pothole_form_sample.py -- 生成"含形面偏差 + 已知凹坑"的仿真样本(验证局部基准口径)

与 make_pothole_sample.py(理想圆柱, 无形面偏差)的区别:
  本样本叠加了**二阶碗形 + 线性倾斜的形面偏差(±1.5mm)**, 用于复现真实件的情形:
    - 口径A(到理想柱面距离) 会把形面偏差当凹塘 -> 失败
    - 口径B(相对局部基准的下凹) 应正确找回已知凹坑

输出:
  PCDfiles/pothole_dent_form.pcd        点云(binary)
  PCDfiles/pothole_dent_form_truth.json 地面真值
"""
import json
import math
import os

import numpy as np

# ---- 与真实样本一致的圆柱基准(与测试程序使用的设计半径 1940 对应) ----
AXIS_PT = np.array([241.98, 42.38, 789.10])
AXIS_DIR = np.array([0.11204, -0.98829, -0.1036])
AXIS_DIR = AXIS_DIR / np.linalg.norm(AXIS_DIR)
R = 1940.0

# ---- 补丁与形面偏差 ----
A_RANGE = (-150.0, 150.0)      # 轴向
PHI_RANGE_DEG = (-5.0, 5.0)    # 周向
DENSITY = 20.0                 # pts/mm^2
NOISE_SIGMA = 0.35             # mm
BOWL_AMP = 1.5                 # 碗形幅度(mm, 二阶)
TILT_AMP = 0.5                 # 线性倾斜幅度(mm)
SEED = 20260911

# ---- 已知凹坑真值(足迹为椭圆半轴: 周向semi_a, 轴向semi_b) ----
DENTS = [
    # 足迹为椭圆半轴(周向 semi_a, 轴向 semi_b); 长轴 30/20/16/11 mm 落在 10~30mm 量级
    dict(name="dent1_30mm", depth=1.5, semi_a=15.0, semi_b=11.0, center_a=-60.0, center_s=70.0),
    dict(name="dent2_20mm", depth=0.8, semi_a=10.0, semi_b=8.0,  center_a=40.0,  center_s=-70.0),
    dict(name="dent3_16mm", depth=0.6, semi_a=8.0,  semi_b=6.0,  center_a=90.0,  center_s=60.0),
    dict(name="dent4_11mm", depth=0.5, semi_a=5.5,  semi_b=4.5,  center_a=-20.0, center_s=-40.0),
]

# ---- 空白区(模拟采样/预处理缺失) ----
BLANKS = [dict(kind="rect", a=(105.0, 145.0), s=(-150.0, -60.0)),
          dict(kind="circle", a=-110.0, s=-90.0, r=10.0)]

a = np.array([1.0, 0.0, 0.0])
if abs(a @ AXIS_DIR) > 0.9:
    a = np.array([0.0, 1.0, 0.0])
T1 = a - AXIS_DIR * (a @ AXIS_DIR); T1 /= np.linalg.norm(T1)
T2 = np.cross(AXIS_DIR, T1)

rng = np.random.default_rng(SEED)
arc_len = R * math.radians(PHI_RANGE_DEG[1] - PHI_RANGE_DEG[0])
n_pts = int((A_RANGE[1] - A_RANGE[0]) * arc_len * DENSITY)
a_x = rng.uniform(A_RANGE[0], A_RANGE[1], n_pts)
phi = rng.uniform(math.radians(PHI_RANGE_DEG[0]), math.radians(PHI_RANGE_DEG[1]), n_pts)
s_y = R * phi

# 形面偏差: 归一化坐标上的二阶碗形 + 线性倾斜(中心化使均值为0)
a_c = 0.5 * (A_RANGE[0] + A_RANGE[1]); a_h = 0.5 * (A_RANGE[1] - A_RANGE[0])
s_c = 0.5 * (math.radians(PHI_RANGE_DEG[0] + PHI_RANGE_DEG[1])) * R
s_h = 0.5 * math.radians(PHI_RANGE_DEG[1] - PHI_RANGE_DEG[0]) * R
u = (a_x - a_c) / a_h
w = (s_y - s_c) / s_h
bowl = BOWL_AMP * ((u * u + w * w) - 2.0 / 3.0)     # 中心化, 期望0
tilt = TILT_AMP * u
form = bowl + tilt
form -= form.mean()

# 凹坑(抛物面剖面, 与既有样本一致)
dent = np.zeros_like(a_x)
for d in DENTS:
    q = ((s_y - d["center_s"]) / d["semi_a"]) ** 2 + ((a_x - d["center_a"]) / d["semi_b"]) ** 2
    dent += np.where(q < 1.0, d["depth"] * (1.0 - q), 0.0)

# 空白区
blank = np.zeros_like(a_x, dtype=bool)
for bk in BLANKS:
    if bk["kind"] == "rect":
        blank |= ((a_x > bk["a"][0]) & (a_x < bk["a"][1]) &
                  (s_y > bk["s"][0]) & (s_y < bk["s"][1]))
    else:
        blank |= ((a_x - bk["a"]) ** 2 + (s_y - bk["s"]) ** 2 < bk["r"] ** 2)
keep = ~blank
a_x, phi, s_y, form, dent = a_x[keep], phi[keep], s_y[keep], form[keep], dent[keep]
n_pts = len(a_x)

r = R + form - dent + rng.normal(0.0, NOISE_SIGMA, n_pts)
pts = (AXIS_PT[None, :]
       + a_x[:, None] * AXIS_DIR[None, :]
       + (r[:, None] * np.cos(phi)[:, None]) * T1[None, :]
       + (r[:, None] * np.sin(phi)[:, None]) * T2[None, :]).astype(np.float32)

out_dir = "PCDfiles"
os.makedirs(out_dir, exist_ok=True)
pcd_path = os.path.join(out_dir, "pothole_dent_form.pcd")
with open(pcd_path, "wb") as f:
    hdr = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\n"
           "SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
           f"WIDTH {n_pts}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n_pts}\nDATA binary\n")
    f.write(hdr.encode("ascii"))
    f.write(pts.tobytes())

truth = dict(
    file=pcd_path, num_points=int(n_pts), design_radius=R,
    axis_point=AXIS_PT.tolist(), axis_dir=AXIS_DIR.tolist(),
    patch=dict(a_range=list(A_RANGE), phi_range_deg=list(PHI_RANGE_DEG), density=DENSITY),
    form_deviation=dict(kind="quadratic_bowl+linear_tilt", bowl_amp=BOWL_AMP, tilt_amp=TILT_AMP,
                        p2p=float(form.max() - form.min()),
                        note="口径A 会把该形面偏差当成凹塘; 口径B 应能分离"),
    dents=[dict(name=d["name"], depth=d["depth"], semi_a=d["semi_a"], semi_b=d["semi_b"],
                center_a=d["center_a"], center_s=d["center_s"],
                false_area_mm2=math.pi * d["semi_a"] * d["semi_b"]) for d in DENTS],
    noise_sigma=NOISE_SIGMA, blanks=BLANKS,
)
with open(os.path.join(out_dir, "pothole_dent_form_truth.json"), "w", encoding="utf-8") as f:
    json.dump(truth, f, ensure_ascii=False, indent=1)

print(f"[OK] {pcd_path}: N={n_pts}")
print(f"     形面偏差: 碗形 ±{BOWL_AMP}mm + 倾斜 ±{TILT_AMP}mm -> 实际 p2p = {form.max()-form.min():.2f} mm")
for d in DENTS:
    print(f"     凹坑 {d['name']}: 深 {d['depth']}mm, 足迹 {2*d['semi_a']:.0f}×{2*d['semi_b']:.0f}mm, "
          f"中心(轴向{d['center_a']}, 周向{d['center_s']})")
print(f"     噪声 σ={NOISE_SIGMA}mm, 真值文件: PCDfiles/pothole_dent_form_truth.json")
