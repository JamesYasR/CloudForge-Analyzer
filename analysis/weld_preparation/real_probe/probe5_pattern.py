#!/usr/bin/env python3
# 采集模式与空洞: 扫描线距 / 内部缺失 / 边界形状 / 占用与残差图(修正图例)
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
U, S = X @ d_ax, X @ d_cur
U -= U.min(); S -= S.min()
cell = 2.0
nu, ns = int(np.ptp(U)/cell)+2, int(np.ptp(S)/cell)+2
iu = np.clip((U/cell).astype(int), 0, nu-1); iss = np.clip((S/cell).astype(int), 0, ns-1)
sums = np.zeros((nu, ns)); cnt = np.zeros((nu, ns))
np.add.at(sums, (iu, iss), r); np.add.at(cnt, (iu, iss), 1.0)
gm = np.where(cnt > 0, sums/np.maximum(cnt, 1), np.nan)

def render(mask_like, cellfun, legend):
    ra = max(1, int(math.ceil(nu/40))); rs = max(1, int(math.ceil(ns/100)))
    P(f"{legend}  (行=轴向, 列=周向, 每格 {ra*cell:.0f}x{rs*cell:.0f} mm; 左=小轴向)")
    for i in range(0, nu, ra):
        P("   |" + "".join(cellfun(i, j, ra, rs) for j in range(0, ns, rs)) + "|")

P("### 占用图: ' '空  ':'稀疏(<40%)  '+'较满  '#'满(>=80%)")
render(None, lambda i, j, ra, rs: (lambda f: ' ' if f == 0 else (':' if f < 0.4 else ('+' if f < 0.8 else '#')))(np.mean(cnt[i:i+ra, j:j+rs] > 0)), "")
P("### 残差图: ' '无数据  '.'|r|<=0.25  '-'<=0.6  '='<=1.2  '#'>1.2 (正负不分, 看数值表)")
render(None, lambda i, j, ra, rs: (lambda v: ' ' if not np.isfinite(v) else ('.' if abs(v) <= 0.25 else ('-' if abs(v) <= 0.6 else ('=' if abs(v) <= 1.2 else '#'))))(np.nanmedian(gm[i:i+ra, j:j+rs])), "")

# 内部缺失: 去掉四周 15mm 后的空缺
b = int(15/cell)
inner = cnt[b:nu-b, b:ns-b]
P(f"内部区域(距四边>15mm) {inner.shape[1]*cell:.0f}x{inner.shape[0]*cell:.0f} mm: 占用率 {100*(inner>0).mean():.1f}%")
# 连通空洞(4邻接) 大小分布
occ = (cnt > 0)
empty = ~occ
lab = np.zeros(empty.shape, int); cur = 0; sizes = []
for i in range(nu):
    for j in range(ns):
        if empty[i, j] and lab[i, j] == 0:
            cur += 1; st = [(i, j)]; lab[i, j] = cur; sz = 0; touches = False
            while st:
                a, c = st.pop(); sz += 1
                if a < b or c < b or a >= nu-b or c >= ns-b: touches = True
                for da, dc in ((1,0),(-1,0),(0,1),(0,-1)):
                    aa, cc = a+da, c+dc
                    if 0 <= aa < nu and 0 <= cc < ns and empty[aa, cc] and lab[aa, cc] == 0:
                        lab[aa, cc] = cur; st.append((aa, cc))
            if not touches: sizes.append(sz)
sizes = np.array(sizes)
if len(sizes):
    P(f"内部空洞: {len(sizes)} 个, 面积中位 {np.median(sizes)*cell*cell:.0f} mm^2, 最大 {sizes.max()*cell*cell:.0f} mm^2, "
      f"总计占内部 {100*sizes.sum()/(inner.size):.2f}%")
else:
    P("内部空洞: 无")
# 扫描线距: 取中心 40x40mm 窗口, 看 U/S 排序后的间隔分布
cu, cs = np.ptp(U)/2, np.ptp(S)/2
w = (np.abs(U-cu) < 20) & (np.abs(S-cs) < 20)
u_w = np.sort(U[w]); s_w = np.sort(S[w])
du = np.diff(u_w); ds = np.diff(s_w)
P(f"中心 40x40mm 窗口 {int(w.sum())} 点")
for nm, d in (("沿轴向 U", du), ("沿周向 S", ds)):
    d = d[d > 1e-4]
    if len(d):
        h, e = np.histogram(d, bins=np.arange(0, min(3.0, d.max()+0.05), 0.05))
        top = np.argsort(h)[-4:][::-1]
        P(f"   {nm} 相邻点间隔: 中位 {np.median(d):.3f} mm; 主要间隔 " +
          ", ".join(f"{e[k]:.2f}mm×{h[k]}" for k in top))
