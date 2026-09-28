#!/usr/bin/env python3
# 读回生成的 far/near.pcd, 按已知柱面框架实测: 足迹/边缘位置/间隙/阶差/噪声/遮蔽带
import os, sys, math, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))

def read_pcd(path):
    with open(path, 'rb') as f:
        data = f.read()
    idx = data.find(b'DATA binary\n')
    hdr = data[:idx].decode('ascii', 'ignore').splitlines()
    n = int([l for l in hdr if l.startswith('POINTS')][0].split()[1])
    off = idx + len(b'DATA binary\n')
    return np.frombuffer(data[off:off + n * 12], dtype='<f4').reshape(n, 3).astype(np.float64)

R = 1940.0
u = np.array([0.37, 0.48, 0.79]); u /= np.linalg.norm(u)
v = np.cross(u, np.array([0.0, 0.0, 1.0])); v /= np.linalg.norm(v)
w = np.cross(u, v); w /= np.linalg.norm(w); O = np.zeros(3)
root = os.path.join(HERE, '..', 'samples_realistic')
P = print
for case in sorted(os.listdir(root)):
    d = os.path.join(root, case)
    if not os.path.isdir(d): continue
    tr = {}
    for l in open(os.path.join(d, 'truth.txt')):
        p = l.split('#')[0].split()
        if len(p) >= 2 and not l.startswith('use:'): tr[p[0]] = p[1]
    g = float(tr['gap_axial_mm']); step = float(tr['step_rational_mm'] if 'step_rational_mm' in tr else tr['step_radial_mm'])
    far = read_pcd(os.path.join(d, 'far.pcd')); near = read_pcd(os.path.join(d, 'near.pcd'))
    psi = float(tr.get('seam_psi_deg', '90')) * math.pi / 180.0
    nx, ny = math.sin(psi), math.cos(psi)
    def ase(Q):
        a = (Q - O) @ u; r = (Q - O) - a[:, None] * u
        s = R * np.arctan2(r @ w, r @ v); e = np.linalg.norm(r, axis=1) - R
        return a * nx + s * ny, a * ny - s * nx, e     # (跨缝x, 沿缝y, 径向e)
    fa, fs, fe = ase(far); na, ns_, ne = ase(near)
    # 边缘位置: 远件最靠近 a=0 的点; 近件最靠近 a=-g(即最大 a)的点
    far_edge = np.quantile(fa, 0.0005); near_edge = np.quantile(na, 0.9995)
    gap_meas = far_edge - near_edge          # 远件边在 x=0, 近件边在 x=-g -> 间隙 = 两者之差
    band_f = (fa > far_edge) & (fa < far_edge + 5); band_n = (na < near_edge) & (na > near_edge - 5)
    dstep = np.median(ne[band_n]) - np.median(fe[band_f])
    cell = 1.0
    key_f = (np.clip((fa - fa.min()) / cell, 0, None).astype(int) * 10000 + np.clip((fs - fs.min()) / cell, 0, None).astype(int))
    uf, inv = np.unique(key_f, return_inverse=True)
    noise_f = np.std(fe - np.array([np.mean(fe[inv == k]) for k in range(len(uf))])[inv])
    span_f = (np.ptp(fa), np.ptp(fs)); span_n = (np.ptp(na), np.ptp(ns_))
    dens = len(far) / (span_f[0] * span_f[1])
    shadow = int(np.sum((fa > 0.05) & (fa < 1.0)))   # 接头边第一格内的点数(应 >0, 说明边未被削)
    P(f"{case}")
    P(f"   点: 远 {len(far):7d} 近 {len(near):7d} | 足迹 远 {span_f[0]:.0f}x{span_f[1]:.0f} 近 {span_n[0]:.0f}x{span_n[1]:.0f} mm | 密度 {dens:.2f} 点/mm² (点距 {math.sqrt(1/dens):.3f})")
    P(f"   真值: 间隙 {g:+.2f} 阶差 {step:+.2f} | 实测: 间隙 {gap_meas:+.3f} (误 {gap_meas-g:+.3f})  阶差 {dstep:+.3f} (误 {dstep-step:+.3f})")
    P(f"   远件边缘 a={far_edge:+.2f} (应≈0)  近件边缘 a={near_edge:+.2f} (应≈{-g:+.2f}) | 点级噪声 σ={noise_f:.4f} | 期望 %s | 接头边缘 5mm 带内点数 远 {int(band_f.sum())} 近 {int(band_n.sum())} | 遮蔽带残留 {shadow}")
