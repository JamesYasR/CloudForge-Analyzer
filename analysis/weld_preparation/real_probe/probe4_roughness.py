#!/usr/bin/env python3
# 真实样本定量表征: 二次型参考面 -> 残差; 点级噪声; 多尺度粗糙度; 残差图/占用图; 台阶段; 尖刺
import sys, os, math, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', '..', 'cylinder_patch')))
from analyze_cylinder_patch import read_pcd_xyz

def box_mean(a, k):
    m = np.isfinite(a); v = np.where(m, a, 0.0)
    S = np.pad(v.cumsum(0).cumsum(1), ((1, 0), (1, 0)))
    N = np.pad(m.astype(float).cumsum(0).cumsum(1), ((1, 0), (1, 0)))
    nu, ns = a.shape
    i0 = np.clip(np.arange(nu) - k // 2, 0, nu); i1 = np.clip(i0 + k, 0, nu)
    j0 = np.clip(np.arange(ns) - k // 2, 0, ns); j1 = np.clip(j0 + k, 0, ns)
    sv = S[np.ix_(i1, j1)] - S[np.ix_(i0, j1)] - S[np.ix_(i1, j0)] + S[np.ix_(i0, j0)]
    cn = N[np.ix_(i1, j1)] - N[np.ix_(i0, j1)] - N[np.ix_(i1, j0)] + N[np.ix_(i0, j0)]
    return np.where(cn > 0, sv / np.maximum(cn, 1), np.nan)

def main():
    pts, _ = read_pcd_xyz(sys.argv[1]); pts = pts[np.isfinite(pts).all(1)]
    P = print; c0 = pts.mean(0); X = pts - c0
    V = np.linalg.eigh(X.T @ X / len(X))[1]
    n, a1, a2 = V[:, 0], V[:, 2], V[:, 1]
    uu, vv, zz = X @ a1, X @ a2, X @ n
    A = np.stack([np.ones_like(uu), uu, vv, uu*uu, uu*vv, vv*vv], 1)
    coef, *_ = np.linalg.lstsq(A, zz, rcond=None)
    r = zz - A @ coef
    H = np.array([[2*coef[3], coef[4]], [coef[4], 2*coef[5]]])
    lam, vec = np.linalg.eigh(H); icur = int(np.argmax(np.abs(lam)))
    R_eff = 1.0 / abs(lam[icur])
    d_cur = vec[0, icur]*a1 + vec[1, icur]*a2; d_cur /= np.linalg.norm(d_cur)
    d_ax = np.cross(n, d_cur); d_ax /= np.linalg.norm(d_ax)
    U, S = X @ d_ax, X @ d_cur
    P(f"### 半径与形面")
    P(f"二次型参考面: R_eff = {R_eff:.1f} mm (正交主曲率 {lam[1-icur]:+.2e} 1/mm)")
    P(f"残差 r: sigma {r.std():.4f} mm, p2p {np.ptp(r):.3f} mm")
    P(f"展布: 轴向 {np.ptp(U):.1f} mm x 周向 {np.ptp(S):.1f} mm; 面密度 {len(pts)/(np.ptp(U)*np.ptp(S)):.2f} 点/mm^2 (等效点距 {math.sqrt(np.ptp(U)*np.ptp(S)/len(pts)):.3f} mm)")
    # 1mm 栅格
    cell = 1.0
    iu = np.clip(((U-U.min())/cell).astype(int), 0, None); iss = np.clip(((S-S.min())/cell).astype(int), 0, None)
    nu, ns = iu.max()+1, iss.max()+1
    sg = np.zeros((nu, ns)); cg = np.zeros((nu, ns))
    np.add.at(sg, (iu, iss), r); np.add.at(cg, (iu, iss), 1.0)
    gm = np.where(cg > 0, sg/np.maximum(cg, 1), np.nan)
    P(f"栅格 1mm: {nu}x{ns}, 占用率 {100*(cg>0).mean():.1f}%")
    # 点级噪声: 点到所属 1mm 格均值的散布
    cellmean_pt = gm[iu, iss]
    P(f"点级散布(相对 1mm 格均值) sigma = {np.nanstd(r - cellmean_pt):.4f} mm")
    # 格内平均点数(含重复采样/多回波提示)
    occ = cg > 0
    P(f"每占用格平均点数 {cg[occ].mean():.1f} (中位 {np.median(cg[occ]):.0f})")
    # 多尺度粗糙度: 相对 k×k 盒均值(1mm 格)的残差 sigma
    P("多尺度粗糙度(相对 k mm 尺度平滑面的 sigma):")
    for k in [2, 3, 5, 10, 20, 50, 100]:
        sm = box_mean(gm, k)
        d = gm - sm
        P(f"   k={k:3d} mm : sigma {np.nanstd(d):.4f} mm, |d|>0.5mm 格占 {100*np.nanmean(np.abs(d)>0.5):.1f}%")
    # 残差图(带符号, 固定 +-0.5/1.5 mm 分档)
    P("残差图(行=轴向, 列=周向; ' '=无数据  '.'=|r|<=0.3  小写=负  大写=正  1..6 对应 0.3/0.6/1.0/1.5/2.5/>2.5 mm):")
    ra = max(1, int(math.ceil(nu/32))); rs = max(1, int(math.ceil(ns/78)))
    def sym(v):
        if not np.isfinite(v): return ' '
        av = abs(v); lv = 0 if av <= 0.3 else (1 if av <= 0.6 else (2 if av <= 1.0 else (3 if av <= 1.5 else (4 if av <= 2.5 else 5))))
        return ' .12345'[lv] if v < 0 else ' .ABCDE'[lv]
    for i in range(0, nu, ra):
        P("   |" + "".join(sym(np.nanmedian(gm[i:i+ra, j:j+rs])) for j in range(0, ns, rs)) + "|")
    # 逐行/逐列中值 + 占用, 找缝(台阶/空缺)
    rocc = np.array([np.mean(occ[i:i+ra]) for i in range(0, nu, ra)])
    rmed = np.array([np.nanmedian(gm[i:i+ra]) for i in range(0, nu, ra)])
    P("逐行(轴向 每" f"{ra}" "mm): 占用率 / 残差中值 mm")
    P("   " + " ".join(f"{o*100:3.0f}/{m:+.2f}" for o, m in zip(rocc, rmed)))
    co = np.array([np.mean(occ[:, j:j+rs]) for j in range(0, ns, rs)])
    cm = np.array([np.nanmedian(gm[:, j:j+rs]) for j in range(0, ns, rs)])
    P("逐列(周向 每" f"{rs}" "mm): 占用率 / 残差中值 mm")
    P("   " + " ".join(f"{o*100:3.0f}/{m:+.2f}" for o, m in zip(co, cm)))
    # 尖刺/孤立点
    dev = r - cellmean_pt
    sp = np.abs(dev) > 1.0
    P(f"孤立偏离点(|点-格均值|>1.0mm): {int(sp.sum())} 个 ({100*sp.mean():.3f}%)")
    if sp.sum():
        P(f"   其偏离量 sigma {dev[sp].std():.2f} mm, 最大 {np.abs(dev[sp]).max():.2f} mm")

main()
