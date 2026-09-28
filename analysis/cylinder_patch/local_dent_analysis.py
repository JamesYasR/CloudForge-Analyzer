#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
local_dent_analysis.py -- 用"局部基准"口径分析真实样本到底有没有凹坑

两种口径对比:
  口径A(需求原文): 深度 = 到理想柱面的距离(径向残差)          -> 形面偏差会污染
  口径B(工程常用): 深度 = 相对"周围正常表面"(邻域大窗口中值)的下凹量 -> 只反映局部凹陷

用法: python3 local_dent_analysis.py <pcd> <R> <ax,ay,az> <px,py,pz> [窗口mm]
"""
import math
import sys
import os
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_cylinder_patch import read_pcd_xyz, ortho_frame
from analyze_raw_with_prior import fit_center_fixed_R


def main():
    cloud = sys.argv[1]
    R = float(sys.argv[2])
    d = np.array([float(v) for v in sys.argv[3].split(',')]); d /= np.linalg.norm(d)
    p0 = np.array([float(v) for v in sys.argv[4].split(',')])
    win_mm = float(sys.argv[5]) if len(sys.argv) > 5 else 80.0
    cell = float(sys.argv[6]) if len(sys.argv) > 6 else 2.0

    pts, _ = read_pcd_xyz(cloud)
    pts = pts[np.isfinite(pts).all(axis=1)]
    _, t1, t2 = ortho_frame(d)
    v = pts - p0
    a = v @ d
    u = v @ t1; w = v @ t2
    # 固定半径拟合圆心(不能用 median: 弧上点的中位点不是圆心!)
    a_c, b_c = fit_center_fixed_R(u, w, R, float(np.mean(u)), float(np.mean(w)), iters=15)
    rho = np.hypot(u - a_c, w - b_c)
    e = rho - R                                   # 口径A: 到理想柱面
    phi = np.arctan2(w - b_c, u - a_c)
    s = R * phi                                   # 周向弧长

    print(f"点云 {os.path.basename(cloud)}  N={len(pts)}, R={R} mm, 局部窗口={win_mm} mm")
    print(f"口径A(到理想柱面): median={np.median(e):+.3f}, p1={np.percentile(e,1):+.2f}, "
          f"p99={np.percentile(e,99):+.2f}, min={e.min():+.2f} mm")

    # ---- 网格化 -> 每格中值 -> 大窗口中值作为"局部基准" ----
    a_lo, a_hi = np.percentile(a, [0.1, 99.9]); s_lo, s_hi = np.percentile(s, [0.1, 99.9])
    na = int((a_hi - a_lo) / cell) + 1
    ns = int((s_hi - s_lo) / cell) + 1
    ia = np.clip(((a - a_lo) / cell).astype(int), 0, na - 1)
    isx = np.clip(((s - s_lo) / cell).astype(int), 0, ns - 1)
    occ = np.zeros((na, ns), bool); occ[ia, isx] = True

    # 每格中值残差
    order = np.argsort(ia.astype(np.int64) * ns + isx)
    keys = ia[order] * ns + isx[order]; vals = e[order]
    uniq, starts = np.unique(keys, return_index=True)
    cell_med = np.full((na, ns), np.nan)
    for k, st in enumerate(starts):
        en = starts[k + 1] if k + 1 < len(starts) else len(vals)
        r_, c_ = divmod(int(uniq[k]), ns)
        cell_med[r_, c_] = np.median(vals[st:en])

    # 大窗口中值滤波(仅用占据格) 作为局部基准
    win = max(3, int(round(win_mm / cell)))
    if win % 2 == 0: win += 1
    half = win // 2
    ref = np.full_like(cell_med, np.nan)
    for r in range(na):
        r0, r1 = max(0, r - half), min(na, r + half + 1)
        for c in range(ns):
            c0, c1 = max(0, c - half), min(ns, c + half + 1)
            blk = cell_med[r0:r1, c0:c1]
            vv = blk[np.isfinite(blk)]
            if vv.size >= 8:
                ref[r, c] = np.median(vv)

    depth = ref - cell_med                        # 正=相对局部基准下凹
    f = np.isfinite(depth)
    print(f"口径B(相对局部基准的下凹量): 最深={depth[f].max():+.3f} mm, "
          f"p1={np.percentile(depth[f],1):+.3f}, p50={np.median(depth[f]):+.3f}, "
          f"p99={np.percentile(depth[f],99):+.3f} mm")
    for thr in (0.5, 1.0, 1.5, 2.0):
        frac = float((depth[f] > thr).mean()) * 100
        print(f"    下凹 > {thr:.1f} mm 的格占比 = {frac:5.2f}%")
    # 局部基准与原值对比: 说明"形面偏差"有多大
    print(f"    局部基准相对理想柱面的偏移范围 = "
          f"{np.nanmin(ref)-np.median(ref):+.2f} .. {np.nanmax(ref)-np.median(ref):+.2f} mm"
          f"  <-- 这就是被口径A当成'凹塘'的形面偏差")

    # 已知坑心处的实测局部深度(对照真值)
    try:
        import json as _json
        tpath = os.path.join(os.path.dirname(cloud) if os.path.dirname(cloud) else '.',
                             'pothole_dent_form_truth.json')
        if not os.path.exists(tpath):
            tpath = os.path.join('PCDfiles', 'pothole_dent_form_truth.json')
        if os.path.exists(tpath):
            truth = _json.load(open(tpath, encoding='utf-8'))
            print("    --- 已知凹坑真值 vs 口径B 实测 ---")
            for dn in truth["dents"]:
                ca, cs = dn["center_a"], dn["center_s"]
                r_ = int((ca - a_lo) / cell); c_ = int((cs - s_lo) / cell)
                if 0 <= r_ < na and 0 <= c_ < ns and np.isfinite(depth[r_, c_]):
                    # 坑心邻域(±3格)中值, 抑制单格噪声
                    blk = depth[max(0, r_-3):r_+4, max(0, c_-3):c_+4]
                    vv = blk[np.isfinite(blk)]
                    meas = float(np.median(vv)) if vv.size else float('nan')
                    print(f"      {dn['name']}: 真值深 {dn['depth']:.2f} mm -> 实测局部深度 "
                          f"{meas:.3f} mm (误差 {meas-dn['depth']:+.3f} mm), "
                          f"足迹真值 {2*dn['semi_a']:.0f}x{2*dn['semi_b']:.0f} mm")
    except Exception as ex:
        print(f"    (真值对照跳过: {ex})")

    # 内部连通凹陷(排除贴边)统计
    for thr in (1.0, 1.5, 2.0):
        m = f & (depth > thr)
        # 去掉贴边格
        mm = m.copy()
        mm[:2, :] = mm[-2:, :] = mm[:, :2] = mm[:, -2:] = False
        lab = np.zeros_like(mm, np.int32); cur = 0; comps = []
        for r in range(na):
            for c in np.where(mm[r])[0]:
                if lab[r, c] == 0:
                    cur += 1; size = 0; dq = deque([(r, c)]); lab[r, c] = cur
                    rs = [r]; cs = [c]
                    while dq:
                        y, x = dq.popleft(); size += 1
                        for dy in (-1, 0, 1):
                            for dx in (-1, 0, 1):
                                ny, nx = y + dy, x + dx
                                if 0 <= ny < na and 0 <= nx < ns and mm[ny, nx] and lab[ny, nx] == 0:
                                    lab[ny, nx] = cur; dq.append((ny, nx)); rs.append(ny); cs.append(nx)
                    comps.append((size, max(rs) - min(rs) + 1, max(cs) - min(cs) + 1))
        comps.sort(reverse=True)
        head = comps[:3]
        print(f"    阈值 {thr:.1f} mm: 内部连通凹陷块 {len(comps)} 个, 最大的: "
              + (", ".join(f"{s}格({dy}×{dx}格≈{dy*cell:.0f}×{dx*cell:.0f}mm)" for s, dy, dx in head)
                 if head else "无"))


if __name__ == "__main__":
    main()
