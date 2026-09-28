#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_pothole_sample.py -- 生成凹塘测量功能的测试点云

基于真实样本 test2/1_cld.pcd 拟合出的圆柱参数(轴/R), 在其上生成:
  - 一块 ~260x300mm 的圆柱面补丁(密度/噪声与真实样本相当)
  - 一个椭圆形凹塘(已知真值: 深度/长短轴)
  - 少量空白区(模拟真实数据缺失)

输出: PCDfiles/pothole_test.pcd (binary)
同时用与 C++ MeasurePothole 相同的算法逻辑做镜像验证, 打印期望测量值.
"""
import math
import struct
import sys
from collections import deque

import numpy as np

# ---- 真实样本拟合结果 (docs/局部圆柱点云模型-特征理解与分析.md) ----
AXIS_PT = np.array([241.98, 42.38, 789.10])
AXIS_DIR = np.array([0.11204, -0.98829, -0.1036])
AXIS_DIR = AXIS_DIR / np.linalg.norm(AXIS_DIR)
R = 1940.0   # 与项目对话框默认设计半径 1940mm 一致, 便于默认参数直接验证

# 正交框架
a = np.array([1.0, 0.0, 0.0])
if abs(a @ AXIS_DIR) > 0.9:
    a = np.array([0.0, 1.0, 0.0])
T1 = a - AXIS_DIR * (a @ AXIS_DIR); T1 /= np.linalg.norm(T1)
T2 = np.cross(AXIS_DIR, T1)

# ---- 凹塘真值定义 ----
DENT_DEPTH = 2.5       # mm 中心深度
DENT_SEMI_A = 45.0     # mm 长半轴(周向)
DENT_SEMI_B = 30.0     # mm 短半轴(轴向)
DENT_CENTER_S = 20.0   # mm 周向位置(展开域)
DENT_CENTER_A = -15.0  # mm 轴向位置

# ---- 补丁定义 ----
A_RANGE = (-150.0, 150.0)   # 轴向
PHI_RANGE_DEG = (-5.0, 5.0)  # 周向角(半径变大后收窄, 保持补丁尺寸相近)
DENSITY = 20.0    # pts/mm^2
NOISE_SIGMA = 0.35  # mm
SEED = 42

rng = np.random.default_rng(SEED)

# ---- 生成点 ----
# 在展开域 (a, s=R*phi) 均匀采样
arc_len = R * math.radians(PHI_RANGE_DEG[1] - PHI_RANGE_DEG[0])
n_pts = int((A_RANGE[1] - A_RANGE[0]) * arc_len * DENSITY)
a_x = rng.uniform(A_RANGE[0], A_RANGE[1], n_pts)
phi = rng.uniform(math.radians(PHI_RANGE_DEG[0]), math.radians(PHI_RANGE_DEG[1]), n_pts)
s_y = R * phi

# 椭圆抛物面凹坑: 深度 = D * (1 - q), q = 椶圆参数(0中心, 1边界)
u = (s_y - DENT_CENTER_S) / DENT_SEMI_A
v = (a_x - DENT_CENTER_A) / DENT_SEMI_B
q = u * u + v * v
dent = np.where(q < 1.0, DENT_DEPTH * (1.0 - q), 0.0)

# 空白区: 两块矩形 + 一个圆孔(模拟标定点)
blank = ((a_x > 40) & (a_x < 85) & (s_y > 60) & (s_y < 130))
u2 = (a_x - 90.0) / 12.0; v2 = (s_y + 80.0) / 12.0
blank |= (u2 * u2 + v2 * v2 < 1.0)

keep = ~blank
a_x, phi, dent = a_x[keep], phi[keep], dent[keep]
n_pts = len(a_x)

# 半径: R - dent + 噪声
r = R - dent + rng.normal(0.0, NOISE_SIGMA, n_pts)
pts = (AXIS_PT[None, :]
       + a_x[:, None] * AXIS_DIR[None, :]
       + (r[:, None] * np.cos(phi)[:, None]) * T1[None, :]
       + (r[:, None] * np.sin(phi)[:, None]) * T2[None, :])
pts = pts.astype(np.float32)

out_path = "PCDfiles/pothole_test.pcd"
with open(out_path, "wb") as f:
    hdr = ("# .PCD v0.7 - Point Cloud Data file format\n"
           "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
           f"WIDTH {n_pts}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
           f"POINTS {n_pts}\nDATA binary\n")
    f.write(hdr.encode("ascii"))
    f.write(pts.tobytes())
print(f"[OK] 生成 {out_path}: N={n_pts}, 密度={DENSITY}/mm^2, 噪声sigma={NOISE_SIGMA}mm")
print(f"     凹塘真值: 深度={DENT_DEPTH}mm, 椭圆足迹 长轴={2*DENT_SEMI_A} 短轴={2*DENT_SEMI_B} mm (中心 周向{DENT_CENTER_S}, 轴向{DENT_CENTER_A})")

# ================= 镜像验证 (与 C++ MeasurePothole 相同逻辑) =================
print("\n---- 镜像验证(与 C++ 相同算法) ----")
# 残差
v = pts.astype(np.float64) - AXIS_PT[None, :]
h = v @ AXIS_DIR
rad = v - h[:, None] * AXIS_DIR[None, :]
rho = np.linalg.norm(rad, axis=1)
e = rho - R
print(f"残差: median={np.median(e):+.4f} robust_sigma={1.4826*np.median(np.abs(e-np.median(e))):.4f} mm")
print(f"最大凹深(期望≈{DENT_DEPTH}): {-e.min():.4f} mm @ {pts[np.argmin(e)].round(2)}")

# 自动阈值
sigma_rob = 1.4826 * np.median(np.abs(e - np.median(e)))
thr = max(3.0 * sigma_rob, 0.8)
print(f"自动阈值: 3*sigma={3*sigma_rob:.3f} -> thr={thr:.3f} mm")

# 凹塘点群
cand = e < -thr
print(f"凹塘候选点(低于阈值): {cand.sum()}")

# 欧氏聚类等价: 3mm 体素 6-连通取最大簇 (镜像 C++ EuclideanClusterExtraction)
vox = 3.0
cand_idx = np.where(cand)[0]
vkey = np.floor(pts[cand] / vox).astype(np.int64)
voxmap = {}
for i, k in enumerate(map(tuple, vkey)):
    voxmap.setdefault(k, []).append(i)
vseen = set(); best_idx = []; 
for vk in voxmap:
    if vk in vseen:
        continue
    comp = []
    dq = deque([vk]); vseen.add(vk)
    while dq:
        u_ = dq.popleft(); comp.extend(voxmap[u_])
        for d0 in ((-1,0,0),(1,0,0),(0,-1,0),(0,1,0),(0,0,-1),(0,0,1)):
            nk = (u_[0]+d0[0], u_[1]+d0[1], u_[2]+d0[2])
            if nk in voxmap and nk not in vseen:
                vseen.add(nk); dq.append(nk)
    if len(comp) > len(best_idx):
        best_idx = comp
pit_sel = cand_idx[np.array(best_idx)]
print(f"最大簇(凹塘点群): {len(pit_sel)} 点")

# 展开 + 栅格 + 边界 + 椭圆拟合 (同 C++, 只用最大簇)
aa = h[pit_sel]
ss = R * np.arctan2(rad[pit_sel] @ T2, rad[pit_sel] @ T1)
amin, amax = aa.min(), aa.max(); smin, smax = ss.min(), ss.max()
da, ds = amax - amin, smax - smin
spacing = math.sqrt(da * ds / len(pit_sel))
cell = max(1.0, 2.2 * spacing)
na = int(da / cell) + 1; ns = int(ds / cell) + 1
ia = np.clip(((aa - amin) / cell).astype(int), 0, na - 1)
iss = np.clip(((ss - smin) / cell).astype(int), 0, ns - 1)
mask = np.zeros((na, ns), bool)
mask[ia, iss] = True

# 最大连通域 (scipy 不可用, 用简化 BFS)
from collections import deque
lab = np.zeros((na, ns), int); cur = 0; best = (0, 0)
for i in range(na):
    for j in range(ns):
        if mask[i, j] and lab[i, j] == 0:
            cur += 1; size = 0; dq = deque([(i, j)]); lab[i, j] = cur
            while dq:
                y, x = dq.popleft(); size += 1
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < na and 0 <= nx < ns and mask[ny, nx] and lab[ny, nx] == 0:
                            lab[ny, nx] = cur; dq.append((ny, nx))
            if size > best[1]: best = (cur, size)
main = lab == best[0]

# 边界格
b = np.zeros_like(main)
b[1:-1, 1:-1] = main[1:-1, 1:-1] & (~main[:-2, 1:-1] | ~main[2:, 1:-1] | ~main[1:-1, :-2] | ~main[1:-1, 2:])
b[0, :] = b[-1, :] = b[:, 0] = b[:, -1] = False
ys, xs = np.where(b)
cx = amin + (ys + 0.5) * cell
cy = smin + (xs + 0.5) * cell
print(f"栅格 {na}x{ns} cell={cell:.2f}mm, 边界点={len(ys)}")

# Fitzgibbon/Halir-Flusser 椭圆拟合(与 C++ 同)
def fit_ellipse(x, y):
    mx, my = x.mean(), y.mean()
    scale = np.sqrt(np.mean((x - mx) ** 2 + (y - my) ** 2))
    xx, yy = (x - mx) / scale, (y - my) / scale
    D1 = np.column_stack([xx * xx, xx * yy, yy * yy])
    D2 = np.column_stack([xx, yy, np.ones_like(xx)])
    S11 = D1.T @ D1; S12 = D1.T @ D2; S22 = D2.T @ D2
    C1 = np.array([[0, 0, 0.5], [0, -1, 0], [0.5, 0, 0]], float)
    S22i = np.linalg.inv(S22)
    M = C1 @ (S11 - S12 @ S22i @ S12.T)
    w, V = np.linalg.eig(M)
    for k in range(3):
        v1 = V[:, k].real
        disc = 4 * v1[0] * v1[2] - v1[1] ** 2
        if disc <= 1e-14: continue
        v1 = v1 / math.sqrt(disc)          # 约束定标: 4AC-B^2 = 1
        a2 = -S22i @ S12.T @ v1
        # 反归一化基量 + 平移常数项(F=+1 约定下特征向量符号任意, s=±1 都试)
        shift = (v1[0]*mx**2 + v1[1]*mx*my + v1[2]*my**2)/scale**2 - (a2[0]*mx + a2[1]*my)/scale
        for sg in (1.0, -1.0):
            A = sg * v1[0] / scale**2
            B = sg * v1[1] / scale**2
            C = sg * v1[2] / scale**2
            D_ = sg * ((-2*v1[0]*mx - v1[1]*my)/scale**2 + a2[0]/scale)
            E_ = sg * ((-2*v1[2]*my - v1[1]*mx)/scale**2 + a2[1]/scale)
            F_ = 1.0 + sg * shift
            if A < 0:                        # 符号归一化(同一曲线乘-1)
                A, B, C, D_, E_, F_ = -A, -B, -C, -D_, -E_, -F_
            if 4*A*C - B*B <= 1e-12: continue
            Mq = np.array([[2*A, B], [B, 2*C]])
            c0 = np.linalg.solve(Mq, np.array([-D_, -E_]))
            F0 = F_ + 0.5*(D_*c0[0] + E_*c0[1])
            if F0 >= -1e-12: continue
            ev, evec = np.linalg.eigh(Mq)
            if ev[0] <= 1e-12: continue
            s1 = math.sqrt(-F0 / ev[0]); s2 = math.sqrt(-F0 / ev[1])
            if s1 > 0 and s2 > 0:
                return c0, s1, s2, 0.5 * math.atan2(B, A - C) + math.pi / 2
    return None

res = fit_ellipse(cx, cy)
if res:
    (ex, ey), s1, s2, ang = res
    print(f"椭圆拟合: 长轴={2*s1:.2f} 短轴={2*s2:.2f} mm, 中心(轴向{ex:.1f}, 周向{ey:.1f})")
    # 理论期望: 抛物面深度 D(1-q), 阈值thr处 q=1-thr/D
    q_edge = 1 - thr / DENT_DEPTH
    f = math.sqrt(q_edge)
    print(f"理论期望(阈值截断): 长轴≈{2*DENT_SEMI_A*f:.1f} 短轴≈{2*DENT_SEMI_B*f:.1f} mm (q_edge={q_edge:.3f})")
else:
    print("椭圆拟合失败")
