#!/usr/bin/env python3
# 缝线检验: (a) 残差场的梯度脊(台阶线)  (b) 缺失点投影直方图的线状峰(缝线)
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
coef, *_ = np.linalg.lstsq(A, zz, rcond=None); r = zz - A @ coef
H = np.array([[2*coef[3], coef[4]], [coef[4], 2*coef[5]]])
lam, vec = np.linalg.eigh(H); icur = int(np.argmax(np.abs(lam)))
d_cur = vec[0, icur]*a1 + vec[1, icur]*a2; d_cur /= np.linalg.norm(d_cur)
d_ax = np.cross(n, d_cur); d_ax /= np.linalg.norm(d_ax)
U = X @ d_ax; S = X @ d_cur
U -= U.min(); S -= S.min()
cell = 2.0
nu, ns = int(np.ptp(U)/cell)+2, int(np.ptp(S)/cell)+2
iu = np.clip((U/cell).astype(int), 0, nu-1); iss = np.clip((S/cell).astype(int), 0, ns-1)
sums = np.zeros((nu, ns)); cnt = np.zeros((nu, ns))
np.add.at(sums, (iu, iss), r); np.add.at(cnt, (iu, iss), 1.0)
gm = np.where(cnt > 0, sums/np.maximum(cnt, 1), np.nan)

# (a) 梯度脊: 用差分的稳健中值梯度
def nanmed(a):
    return np.nanmedian(a) if np.isfinite(a).any() else np.nan
gu = np.full_like(gm, np.nan); gs = np.full_like(gm, np.nan)
for i in range(1, nu-1):
    for j in range(1, ns-1):
        if np.isfinite(gm[i, j]):
            gu[i, j] = nanmed(gm[i+1, max(0,j-1):j+2]) - nanmed(gm[i-1, max(0,j-1):j+2])
            gs[i, j] = nanmed(gm[max(0,i-1):i+2, j+1]) - nanmed(gm[max(0,i-1):i+2, j-1])
gmagn = np.hypot(gu, gs) / (2*cell)
P(f"### 残差梯度(台阶敏感): 中位 {np.nanmedian(gmagn):.4f} mm/mm, 95% {np.nanpercentile(gmagn,95):.4f}, 最大 {np.nanmax(gmagn):.4f}")
mx = np.nanmax(gmagn)
P("梯度图( ' '空  '.'<=0.2  1..6 = 0.2/0.4/0.6/1/2/>2 倍中位; 找线状脊 ):")
ra = max(1, int(math.ceil(nu/40))); rs = max(1, int(math.ceil(ns/100)))
for i in range(0, nu, ra):
    row = []
    for j in range(0, ns, rs):
        v = np.nanmax(gmagn[i:i+ra, j:j+rs]) if np.isfinite(gmagn[i:i+ra, j:j+rs]).any() else np.nan
        if not np.isfinite(v): row.append(' ')
        elif v <= 0.2: row.append('.')
        else: row.append('123456'[min(5, int(6*(v-0.2)/max(1e-9, mx-0.2)))])
    P("   |" + "".join(row) + "|")

# (b) 缺失线检验: 内部空格的投影直方图峰
b = int(15/cell)
occ = cnt > 0
inner = np.zeros_like(occ); inner[b:nu-b, b:ns-b] = True
miss = inner & ~occ
ys, xs = np.nonzero(miss)
wc = np.stack([ (ys+0.5)*cell, (xs+0.5)*cell ], 1)
best = None
for th in range(0, 180, 1):
    d = np.array([math.cos(math.radians(th)), math.sin(math.radians(th))])
    p = wc @ d
    h, e = np.histogram(p, bins=np.arange(p.min(), p.max()+2, 2.0))
    base = h.mean()
    peak = h.max()
    if best is None or peak > best[0]:
        best = (peak, th, base, peak/max(base,1e-9))
P(f"### 缺失点投影直方图最尖峰: 方向 {best[1]}° (0=轴向), 峰值 {best[0]} 格, 均值 {best[2]:.1f}, 峰/均 = {best[3]:.2f}")
P("(若缝是一条缺失线, 该比值应显著 >2; 随机空洞则接近 1.5~2)")
