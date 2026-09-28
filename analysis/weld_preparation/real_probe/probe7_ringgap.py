#!/usr/bin/env python3
# 在"深度内部"(腐蚀掉足迹边界)里找环向缺失带: 逐轴向位置的占用率剖面
import sys, os, math, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', '..', 'cylinder_patch')))
from analyze_cylinder_patch import read_pcd_xyz

pts, _ = read_pcd_xyz(sys.argv[1]); pts = pts[np.isfinite(pts).all(1)]
P = print; c0 = pts.mean(0); X = pts - c0
V = np.linalg.eigh(X.T @ X / len(X))[1]
n, a1, a2 = V[:, 0], V[:, 2], V[:, 1]
uu, vv, zz = X @ a1, X @ a2, X @ n
A = np.stack([np.ones_like(uu), uu, vv, uu*uu, uu*vv, vv*vv], 1)
coef, *_ = np.linalg.lstsq(A, zz, rcond=None)
H = np.array([[2*coef[3], coef[4]], [coef[4], 2*coef[5]]])
lam, vec = np.linalg.eigh(H); icur = int(np.argmax(np.abs(lam)))
d_cur = vec[0, icur]*a1 + vec[1, icur]*a2; d_cur /= np.linalg.norm(d_cur)
d_ax = np.cross(n, d_cur); d_ax /= np.linalg.norm(d_ax)
U = X @ d_ax; S = X @ d_cur; U -= U.min(); S -= S.min()
cell = 1.0
nu, ns = int(np.ptp(U)/cell)+2, int(np.ptp(S)/cell)+2
iu = np.clip((U/cell).astype(int), 0, nu-1); iss = np.clip((S/cell).astype(int), 0, ns-1)
cnt = np.zeros((nu, ns)); np.add.at(cnt, (iu, iss), 1.0)
occ = cnt > 0
# 腐蚀: 半径 r 内无空格的格才算"深度内部"
def erode(mask, r):
    e = ~mask
    S = np.pad(e.cumsum(0).cumsum(1).astype(float), ((1,0),(1,0)))
    i0 = np.clip(np.arange(mask.shape[0])-r, 0, mask.shape[0]); i1 = np.clip(i0+2*r+1, 0, mask.shape[0])
    j0 = np.clip(np.arange(mask.shape[1])-r, 0, mask.shape[1]); j1 = np.clip(j0+2*r+1, 0, mask.shape[1])
    bad = S[np.ix_(i1,j1)] - S[np.ix_(i0,j1)] - S[np.ix_(i1,j0)] + S[np.ix_(i0,j0)]
    return (bad == 0)
for r in (4, 8, 15):
    deep = erode(occ, r)
    if deep.sum() < 100: continue
    prof = np.array([occ[i:i+1, deep[i]].mean() if deep[i].any() else np.nan for i in range(nu)])
    missing = np.array([(~occ[i, deep[i]]).sum() if deep[i].any() else 0 for i in range(nu)])
    tot = int(missing.sum())
    P(f"腐蚀半径 {r}mm: 深度内部 {int(deep.sum())} 格, 其中空缺 {tot} 格 ({100*tot/max(1,deep.sum()):.2f}%)")
    if tot:
        idx = np.argsort(missing)[-8:][::-1]
        P("   空缺最多的轴向位置(mm)/空缺格数/该处占用率: " +
          ", ".join(f"{i*cell:.0f}mm:{missing[i]}({100*prof[i]:.0f}%)" for i in idx if missing[i] > 0))
        # 空缺的轴向分布集中度
        w = missing / max(1, tot)
        eff = 1.0 / np.sum(w[w > 0] ** 2)
        P(f"   空缺轴向分布: 有效宽度 {eff:.0f} mm (共 {nu} mm) -> {'集中(像缝线)' if eff < 0.25*nu else '分散(像随机空洞/边缘)'}")
# 逐周向位置的占用率(反向检验)
deep = erode(occ, 8)
profc = np.array([occ[deep[:, j], j:j+1].mean() if deep[:, j].any() else np.nan for j in range(ns)])
mc = np.array([(~occ[deep[:, j], j]).sum() if deep[:, j].any() else 0 for j in range(ns)])
idx = np.argsort(mc)[-6:][::-1]
P("腐蚀8mm后, 空缺最多的周向位置(mm)/格数: " + ", ".join(f"{j*cell:.0f}:{mc[j]}" for j in idx))
