#!/usr/bin/env python3
# 打高度图: 把点投到"零曲率方向(轴向) x 曲率方向(周向)"平面, 1mm 栅格取中值, ASCII 显示
import sys, os, math, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', '..', 'cylinder_patch')))
from analyze_cylinder_patch import read_pcd_xyz

pts, _ = read_pcd_xyz(sys.argv[1])
pts = pts[np.isfinite(pts).all(1)]
P = print
c0 = pts.mean(0); X = pts - c0
C = X.T @ X / len(X); w, V = np.linalg.eigh(C)
n = V[:, 0]; a1 = V[:, 2]; a2 = V[:, 1]
uu, vv, zz = X @ a1, X @ a2, X @ n
A = np.stack([np.ones_like(uu), uu, vv, uu*uu, uu*vv, vv*vv], 1)
coef, *_ = np.linalg.lstsq(A, zz, rcond=None)
H = np.array([[2*coef[3], coef[4]], [coef[4], 2*coef[5]]])
lam, vec = np.linalg.eigh(H)
i_cur = int(np.argmax(np.abs(lam))); i_ax = 1 - i_cur
d_cur = vec[0, i_cur]*a1 + vec[1, i_cur]*a2     # 曲率方向(周向)
d_ax  = vec[0, i_ax]*a1 + vec[1, i_ax]*a2       # 零曲率方向(轴向)
d_cur /= np.linalg.norm(d_cur)
d_ax = np.cross(n, d_cur); d_ax /= np.linalg.norm(d_ax)   # 切平面内且垂直于曲率方向
P(f"轴向(零曲率) {np.round(d_ax,4)}   周向(曲率) {np.round(d_cur,4)}   法向 {np.round(n,4)}")
U = X @ d_ax; S = X @ d_cur; Z = X @ n
P(f"展布: 轴向 {np.ptp(U):.1f} mm x 周向 {np.ptp(S):.1f} mm, 平面法向厚度 {np.ptp(Z):.2f} mm")
# 去平面倾斜后看高度结构
B = np.stack([np.ones_like(U), U, S], 1)
cb, *_ = np.linalg.lstsq(B, Z, rcond=None)
Hh = Z - B @ cb
P(f"去倾斜后高度: p2p {np.ptp(Hh):.2f} mm, 直方图(1mm 箱, 只列非零):")
h, edges = np.histogram(Hh, bins=np.arange(np.floor(Hh.min()), np.ceil(Hh.max())+1, 1.0))
for cnt, e0 in zip(h, edges):
    if cnt: P(f"   {e0:+6.0f}..{e0+1:+6.0f} mm : {cnt:7d} {'#'*int(60*cnt/h.max())}")
# 1mm 栅格取中值 -> ASCII 高度图
cell = 2.0
iu = np.clip(((U - U.min())/cell).astype(int), 0, None)
iss = np.clip(((S - S.min())/cell).astype(int), 0, None)
nu, ns = iu.max()+1, iss.max()+1
acc = np.full((nu, ns), np.nan)
for i in range(nu):
    m = iu == i
    if not m.any(): continue
    j = iss[m]; z = Hh[m]
    order = np.argsort(j); j, z = j[order], z[order]
    st = np.searchsorted(j, np.arange(ns)); en = np.searchsorted(j, np.arange(ns)+1)
    for k in range(ns):
        if en[k] > st[k]: acc[i, k] = np.median(z[st[k]:en[k]])
P(f"高度图(行=轴向 每2mm, 列=周向 每2mm; 符号按高度分档, 空格=无数据): 高度范围 {np.nanmin(acc):.2f}..{np.nanmax(acc):.2f} mm")
chars = " .:-=+*#%@"
vmin, vmax = np.nanpercentile(acc, 1), np.nanpercentile(acc, 99)
ra = max(1, int(math.ceil(nu/34))); rs = max(1, int(math.ceil(ns/78)))
for i in range(0, nu, ra):
    row = []
    for j in range(0, ns, rs):
        blk = acc[i:i+ra, j:j+rs]
        v = np.nanmedian(blk)
        row.append(' ' if not np.isfinite(v) else chars[min(9, max(0, int(9*(v-vmin)/max(1e-9, vmax-vmin))))])
    P("   |" + "".join(row) + "|")
