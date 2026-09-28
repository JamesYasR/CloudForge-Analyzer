#!/usr/bin/env python3
# 足迹形状与朝向: 最小面积外接矩形 + 四边直线度(找接头边)
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
U = X @ d_ax; S = X @ d_cur
# 1mm 占有格中心
cell = 1.0
iu = np.clip(((U-U.min())/cell).astype(int), 0, None); iss = np.clip(((S-S.min())/cell).astype(int), 0, None)
nu, ns = iu.max()+1, iss.max()+1
occ = np.zeros((nu, ns), bool); occ[iu, iss] = True
cy, cx = np.nonzero(occ)
uu_c = (cy+0.5)*cell; ss_c = (cx+0.5)*cell
area = len(cy)*cell*cell
best = None
for th in np.arange(0, 180, 0.5):
    t = math.radians(th); p = uu_c*math.cos(t) + ss_c*math.sin(t); q = -uu_c*math.sin(t) + ss_c*math.cos(t)
    A_box = (p.max()-p.min()) * (q.max()-q.min())
    if best is None or A_box < best[0]: best = (A_box, th, p.max()-p.min(), q.max()-q.min())
P(f"足迹: 占用 {len(cy)} 格 = {area:.0f} mm^2")
P(f"最小外接矩形: 角度 {best[1]:.1f}° (相对轴向), 尺寸 {best[2]:.0f} x {best[3]:.0f} mm, "
  f"面积 {best[0]:.0f} mm^2 -> 填充率 {100*area/best[0]:.0f}%  ({'矩形' if area/best[0]>0.85 else ('圆角矩形' if area/best[0]>0.7 else '椭圆/菱形')})")
# 沿最小矩形的两边统计边界(2mm 带), 检查每条边的直线度与点数密度衰减
t = math.radians(best[1])
P_ = uu_c*math.cos(t) + ss_c*math.sin(t); Q_ = -uu_c*math.sin(t) + ss_c*math.cos(t)
for nm, axis in (("短边方向(跨缝?)", P_), ("长边方向(沿缝?)", Q_)):
    h, e = np.histogram(axis, bins=np.arange(axis.min(), axis.max()+2, 2.0))
    edge_in = e[np.argmax(h > 0.2*h.max())] if (h > 0.2*h.max()).any() else e[0]
    P(f"{nm}: 起止 {axis.min():.0f}..{axis.max():.0f} mm, 边缘 3 格密度 {h[:3].tolist()} ... {h[-3:].tolist()}, 中段中位 {np.median(h[3:-3]):.0f} 格/2mm")
# 周边点数密度衰减(看边缘是否"撕裂/稀疏")
b = 3
inner = occ[b:nu-b, b:ns-b]
P(f"边界 3mm 带占用率 {100*occ[:b].mean():.0f}/{100*occ[-b:].mean():.0f} (轴向两端), "
  f"{100*occ[:, :b].mean():.0f}/{100*occ[:, -b:].mean():.0f} (周向两端); 内部 {100*inner.mean():.0f}%")
