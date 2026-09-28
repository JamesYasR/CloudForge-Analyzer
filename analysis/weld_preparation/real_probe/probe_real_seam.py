#!/usr/bin/env python3
# 真实焊缝样本特征提取(用于设计仿真模型): 圆柱基准/展开/缝走向/阶差/间隙/噪声/形面/飞点
import sys, os, math, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', '..', 'cylinder_patch')))
from analyze_cylinder_patch import read_pcd_xyz, ortho_frame, refine_axis, fit_cyl_to_points


def ascii_map(mask, max_cols=78, max_rows=30):
    na, ns = mask.shape
    ra = max(1, int(math.ceil(na / max_rows)))
    rs = max(1, int(math.ceil(ns / max_cols)))
    out = []
    for i in range(0, na, ra):
        row = []
        for j in range(0, ns, rs):
            blk = mask[i:i + ra, j:j + rs]
            f = blk.mean()
            row.append('.' if f < 0.02 else (':' if f < 0.35 else ('+' if f < 0.75 else '#')))
        out.append(''.join(row))
    return out


def main():
    path = sys.argv[1]
    P = print
    pts, _ = read_pcd_xyz(path)
    pts = pts[np.isfinite(pts).all(1)]
    P(f"### 文件 {os.path.basename(path)}")
    P(f"点数 {len(pts)}")
    lo, hi = pts.min(0), pts.max(0)
    P(f"包围盒 {np.round(hi - lo, 1)} mm  对角线 {np.linalg.norm(hi - lo):.1f} mm")
    c0 = pts.mean(0); X = pts - c0
    C = X.T @ X / len(X); w, V = np.linalg.eigh(C)
    nrm, e1, e2 = V[:, 0], V[:, 2], V[:, 1]
    ext = [np.ptp(X @ e1), np.ptp(X @ e2)]
    P(f"PCA 展布: 面内1 {ext[0]:.1f} mm, 面内2 {ext[1]:.1f} mm, 径向厚度 {np.ptp(X @ nrm):.2f} mm")
    seed = e1 if ext[0] > ext[1] else e2
    sub = pts[::max(1, len(pts) // 150000)]
    d, obj, trace = refine_axis(sub, c0, seed)
    fit = fit_cyl_to_points(pts, c0, d)
    ap = c0 + fit['a'] * fit['t1'] + fit['b'] * fit['t2']
    v = pts - ap
    ax = v @ d
    t1, t2 = fit['t1'], fit['t2']
    ph = np.arctan2(v @ t2, v @ t1)
    R = fit['R']; rho = np.hypot(v @ t1, v @ t2); e = rho - R
    P(f"圆柱(自由半径): R = {R:.1f} mm   残差RMS(95%) {fit['obj']:.4f} mm   轴与种子夹角 {math.degrees(math.acos(min(1,abs(float(d@seed))))):.2f}°")
    P(f"轴向展布 {np.ptp(ax):.1f} mm   周向弧长展布 {R*abs(np.ptp(ph)):.1f} mm   角度跨度 {math.degrees(abs(np.ptp(ph))):.2f}°")
    # 低频形面 vs 高频噪声
    s = R * (ph - float(np.median(ph)))
    A = np.stack([np.ones_like(ax), ax, s, ax*ax, ax*s, s*s], 1)
    coef, *_ = np.linalg.lstsq(A, e, rcond=None)
    form = A @ coef; res = e - form
    P(f"径向 2 阶形面项 p2p {np.ptp(form):.3f} mm   去形面后残差 sigma {res.std():.4f} mm  p2p {np.ptp(res):.3f} mm")
    # 密度/点距
    area = np.ptp(ax) * R * abs(np.ptp(ph))
    P(f"面密度 {len(pts)/area:.2f} 点/mm^2 -> 等效点距 {math.sqrt(area/len(pts)):.3f} mm")
    # 栅格占用图
    cell = 2.0
    ia = np.clip(((ax - ax.min()) / cell).astype(int), 0, None)
    iss = np.clip(((s - s.min()) / cell).astype(int), 0, None)
    na, ns = ia.max() + 1, iss.max() + 1
    occ = np.zeros((na, ns), bool); occ[ia, iss] = True
    P(f"栅格 {cell}mm: {na} 行(轴向) x {ns} 列(周向), 占用率 {occ.mean()*100:.1f}%")
    P("占用图(行=轴向 a, 列=周向 s; #占用 +较满 :较少 .空):")
    for line in ascii_map(occ): P("   " + line)
    # 逐行/逐列占用率 -> 判断缝走向
    rowf = occ.mean(1); colf = occ.mean(0)
    P(f"每行占用率: 最低 {rowf.min()*100:.0f}% 最高 {rowf.max()*100:.0f}%   "
      f"每列占用率: 最低 {colf.min()*100:.0f}% 最高 {colf.max()*100:.0f}%")
    # 找占用率显著低于中值的连续带(缝)
    def bands(frac):
        med = np.median(frac)
        low = frac < 0.6 * med
        out, i = [], 0
        while i < len(low):
            if low[i]:
                j = i
                while j + 1 < len(low) and low[j + 1]: j += 1
                out.append((i, j, med))
                i = j + 1
            else: i += 1
        return out
    P(f"低占用带(行/轴向): {[(round(a*cell,1), round(b*cell,1)) for a,b,_ in bands(rowf)]}")
    P(f"低占用带(列/周向): {[(round(a*cell,1), round(b*cell,1)) for a,b,_ in bands(colf)]}")
    np.save(os.path.join(HERE, 'probe_cache.npy'),
            np.stack([ax, s, e, res, form], 1))
    P("(展开坐标已缓存 probe_cache.npy: 列=[a_mm, s_mm, e_mm, 高频残差, 形面项])")


main()
