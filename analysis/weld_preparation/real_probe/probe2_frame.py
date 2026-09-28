#!/usr/bin/env python3
# 用"切平面二次型"定曲率半径与轴向(不受小弧段轴线病态影响), 再重建展开系并找缝
import sys, os, math, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', '..', 'cylinder_patch')))
from analyze_cylinder_patch import read_pcd_xyz, ortho_frame, fit_cyl_to_points
from probe_real_seam import ascii_map

pts, _ = read_pcd_xyz(sys.argv[1])
pts = pts[np.isfinite(pts).all(1)]
P = print
c0 = pts.mean(0); X = pts - c0
C = X.T @ X / len(X); w, V = np.linalg.eigh(C)
n = V[:, 0]; u_ax = V[:, 2]; v_ax = V[:, 1]
uu, vv, zz = X @ u_ax, X @ v_ax, X @ n
A = np.stack([np.ones_like(uu), uu, vv, uu*uu, uu*vv, vv*vv], 1)
coef, *_ = np.linalg.lstsq(A, zz, rcond=None)
zfit = A @ coef; resid = zz - zfit
H = np.array([[2*coef[3], coef[4]], [coef[4], 2*coef[5]]])
lam, vec = np.linalg.eigh(H)
k_major = lam[np.argmax(np.abs(lam))]; k_minor = lam[np.argmin(np.abs(lam))]
R_est = 1.0/abs(k_major)
P(f"### 切平面二次型(基面残差 p2p {np.ptp(zz):.2f} mm, 二次型 p2p {np.ptp(zfit):.2f} mm)")
P(f"主曲率 {k_major:.3e} / {k_minor:.3e} 1/mm -> R ≈ {R_est:.1f} mm (曲率方向 {vec[:, int(np.argmax(np.abs(lam)))]})")
P(f"零曲率方向(≈轴向, 切平面内) {vec[:, int(np.argmin(np.abs(lam)))]}")
# 用该轴向做种子, 固定不同 R 拟合, 看残差-半径曲线
d_seed = (vec[0, int(np.argmin(np.abs(lam)))] * u_ax + vec[1, int(np.argmin(np.abs(lam)))] * v_ax)
d_seed /= np.linalg.norm(d_seed)
_, t1s, t2s = ortho_frame(d_seed)
P("固定半径拟合(对角线法, 同一轴向种子):")
for Rt in [500, 800, 1071, 1400, 1940, 2600, 3500, 5000]:
    v = pts - c0; u = v @ t1s; w2 = v @ t2s
    a0, b0 = float(np.median(u)), float(np.median(w2))
    # 固定R圆心GN
    a, b = a0, b0
    for _ in range(30):
        du, dw = u - a, w2 - b; rho = np.hypot(du, dw) + 1e-12
        res = rho - Rt; j0, j1 = -du/rho, -dw/rho
        h00, h01, h11 = float(j0@j0), float(j0@j1), float(j1@j1)
        g0, g1 = float(j0@res), float(j1@res)
        det = h00*h11 - h01*h01
        if abs(det) < 1e-12: break
        da = (h01*g1 - h11*g0)/det; db = (h01*g0 - h00*g1)/det
        a += da; b += db
        if math.hypot(da, db) < 1e-6: break
    e = np.hypot(u-a, w2-b) - Rt
    P(f"   R={Rt:5d}  e: sigma {e.std():.3f}  p2p {np.ptp(e):.2f}  median {np.median(e):+.3f} mm")
