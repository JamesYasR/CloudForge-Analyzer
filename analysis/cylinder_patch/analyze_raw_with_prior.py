#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_raw_with_prior.py -- 用已拟合的轴/设计半径作先验, 分析原始(未清洗)点云

模拟程序真实流水线的条件: 初始直线已知(MeasureCylindricity::setInitialLine)
+ 设计半径已知(setDesignRadius). 在固定 R 下只优化 (方向2 + 圆心2) 自由度,
然后输出原始点云的展开面残差/占据图与焊缝带、标定点凸起检测.

!! 实测发现(2026-09, test2 样本): 1.pcd(原始) 与 1_cld.pcd(清洗) 不在同一
   配准系(原始点到清洗拟合圆柱轴线的距离中位数仅 ~62mm, 而 1_cld 全部位于
   R≈1260mm 圆柱上; 刚体变换保持距离不变, 故两帧之间必有重注册/非刚体差异).
   因此对 1.pcd 使用 1_cld.pcd 的轴先验无效 —— 本脚本对 test2 运行会在
   预过滤处保留 0 点. 保留本脚本供"同帧"数据使用, 或先完成配准再用.

用法:
    python3 analyze_raw_with_prior.py <raw.pcd> <out_dir> \
        <axis_x,axis_y,axis_z> <center_x,center_y,center_z> <R_mm>
"""
import json
import math
import os
import sys
from collections import deque

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_cylinder_patch import (read_pcd_xyz, ortho_frame, axis_with_tilt,
                                    save_color_map, save_gray_mask,
                                    save_line_plot, interp_nan, smooth1d,
                                    label_components)


def fit_center_fixed_R(u, w, R, a0, b0, iters=12):
    """固定半径, Gauss-Newton 只精修圆心(与 C++ fitCircleCenterFixedRadius 同思路)."""
    a, b = float(a0), float(b0)
    for _ in range(iters):
        du, dw = u - a, w - b
        rho = np.hypot(du, dw) + 1e-12
        res = rho - R
        j0, j1 = -du / rho, -dw / rho
        h00, h01, h11 = float(j0 @ j0), float(j0 @ j1), float(j1 @ j1)
        g0, g1 = float(j0 @ res), float(j1 @ res)
        det = h00 * h11 - h01 * h01
        if abs(det) < 1e-12:
            break
        da = (h01 * g1 - h11 * g0) / det
        db = (h01 * g0 - h00 * g1) / det
        a += da
        b += db
        if math.hypot(da, db) < 1e-5:
            break
    return a, b


def cyl_obj_fixed_R(pts, center, d, R, t1=None, t2=None):
    if t1 is None:
        _, t1, t2 = ortho_frame(d)
    v = pts - center
    u = v @ t1
    w = v @ t2
    a0 = float(np.mean(u))
    b0 = float(np.mean(w))
    a, b = fit_center_fixed_R(u, w, R, a0, b0)
    e = np.hypot(u - a, w - b) - R
    k = max(1, int(0.90 * len(e)))
    part = np.partition(np.abs(e), k - 1)[:k]
    return float(np.sqrt(np.mean(part ** 2))), a, b, t1, t2


def refine_axis_fixed_R(sub, center, d0, R, schedule=((2.0, 0.25), (0.5, 0.05), (0.15, 0.02))):
    d = d0 / np.linalg.norm(d0)
    trace = []
    for rng_deg, step_deg in schedule:
        for _ in range(4):
            _, t1, t2 = ortho_frame(d)
            base = cyl_obj_fixed_R(sub, center, d, R)[0]
            best = (base, 0.0, 0.0)
            grid = np.arange(-rng_deg, rng_deg + 1e-9, step_deg)
            for al_d in grid:
                for be_d in grid:
                    if al_d == 0.0 and be_d == 0.0:
                        continue
                    dd = d + math.tan(math.radians(al_d)) * t1 + math.tan(math.radians(be_d)) * t2
                    o = cyl_obj_fixed_R(sub, center, dd, R)[0]
                    if o < best[0]:
                        best = (o, math.radians(al_d), math.radians(be_d))
            d = d + math.tan(best[1]) * t1 + math.tan(best[2]) * t2
            d /= np.linalg.norm(d)
            trace.append((round(math.degrees(best[1]), 3), round(math.degrees(best[2]), 3),
                          round(best[0], 4)))
            if best[1] == 0.0 and best[2] == 0.0:
                break
    return d, trace


def main():
    raw_path = sys.argv[1]
    out = sys.argv[2]
    ax, ay, az = (float(v) for v in sys.argv[3].split(","))
    cx, cy, cz = (float(v) for v in sys.argv[4].split(","))
    R_design = float(sys.argv[5])
    os.makedirs(out, exist_ok=True)
    rep = []
    P = rep.append

    pts, n_hdr = read_pcd_xyz(raw_path)
    pts = pts[np.isfinite(pts).all(axis=1)]
    N = len(pts)
    P(f"原始点云: {raw_path}  N={N}")
    P(f"先验: 轴=({ax:.5f},{ay:.5f},{az:.5f}) 参考点=({cx:.2f},{cy:.2f},{cz:.2f}) R={R_design:.2f}mm")

    c0 = np.array([cx, cy, cz])
    d0 = np.array([ax, ay, az])

    # 去粗大离群: 只保留与先验圆柱距离 < 60mm 的点(保护焊缝, 去掉飞点)
    _, t10, t20 = ortho_frame(d0)
    v0 = pts - c0
    u0 = v0 @ t10
    w0 = v0 @ t20
    a0, b0 = float(np.median(u0)), float(np.median(w0))
    rho0 = np.hypot(u0 - a0, w0 - b0)
    keep = np.abs(rho0 - R_design) < 60.0
    P(f"预过滤(|rho-R|<60mm): 保留 {int(keep.sum())}/{N} ({keep.mean() * 100:.1f}%)")
    pts = pts[keep]
    N = len(pts)

    sub = pts[::max(1, N // 300000)]
    d, trace = refine_axis_fixed_R(sub, c0, d0, R_design)
    P(f"定半径轴精化轨迹: {trace}")
    P(f"精化轴 d = {d.round(5)} (与先验夹角 "
      f"{math.degrees(math.acos(min(1.0, abs(float(d @ d0))))):.3f}°)")

    _, t1, t2 = ortho_frame(d)
    v = pts - c0
    u = v @ t1
    w = v @ t2
    a0, b0 = float(np.mean(u)), float(np.mean(w))
    a, b = fit_center_fixed_R(u, w, R_design, a0, b0, iters=15)
    e = np.hypot(u - a, w - b) - R_design
    axis_pt = c0 + a * t1 + b * t2
    t_ax = v @ d
    phi = np.arctan2(w - b, u - a)
    lo_t, hi_t = np.percentile(t_ax, [0.05, 99.95])
    ph = np.sort(phi)
    gaps = np.diff(ph)
    wrap_gap = (ph[0] + 2 * math.pi) - ph[-1]
    max_gap = max(float(gaps.max()), wrap_gap)
    span = 2 * math.pi - max_gap
    start = ph[-1] if max_gap == wrap_gap else ph[int(np.argmax(gaps)) + 1]
    phi_c = start + span / 2

    med = float(np.median(e))
    sig = 1.4826 * float(np.median(np.abs(e - med)))
    P(f"全量定半径拟合: R={R_design:.2f}, 截尾RMS(90%)="
      f"{float(np.sqrt(np.mean(np.partition(np.abs(e), int(0.9 * len(e)))[:int(0.9 * len(e))] ** 2))):.3f}mm")
    P(f"残差: median={med:.3f} robust sigma={sig:.3f} "
      f"p95={np.percentile(e, 95):.2f} p99={np.percentile(e, 99):.2f} "
      f"max={e.max():.2f} min={e.min():.2f} mm")
    P(f"轴向范围 L={hi_t - lo_t:.1f}mm, 周向覆盖角={math.degrees(span):.2f}°, "
      f"弧长={R_design * span:.1f}mm")
    P("")

    # 展开面
    phi_r = (phi - phi_c + 3 * math.pi) % (2 * math.pi) - math.pi
    ph_lo, ph_hi = np.percentile(phi_r, [0.05, 99.95])
    cell_u = max((hi_t - lo_t) / 900, 1.0)
    cell_p = max((ph_hi - ph_lo) / 900, 1.0 / R_design)
    nu = int(np.ceil((hi_t - lo_t) / cell_u)) + 1
    npp = int(np.ceil((ph_hi - ph_lo) / cell_p)) + 1
    iu = np.clip(((t_ax - lo_t) / cell_u).astype(int), 0, nu - 1)
    ip = np.clip(((phi_r - ph_lo) / cell_p).astype(int), 0, npp - 1)
    occ = np.zeros((nu, npp), bool)
    occ[iu, ip] = True
    sumg = np.zeros((nu, npp))
    cntg = np.zeros((nu, npp), np.int32)
    np.add.at(sumg, (iu, ip), e)
    np.add.at(cntg, (iu, ip), 1)
    egrid = np.where(cntg > 0, sumg / np.maximum(cntg, 1), np.nan)

    vmax = 8.0
    save_color_map(np.where(occ, np.nan_to_num(egrid, 0.0), 0.0),
                   os.path.join(out, "r1_raw_residual.png"), vmax, scale=2)
    save_gray_mask(occ, os.path.join(out, "r2_raw_occupancy.png"), scale=2)
    P(f"展开面 {nu}x{npp} (cell={cell_u:.2f}mm x {math.degrees(cell_p):.3f}°), 色标±{vmax}mm")

    # 行/列剖面 + 焊缝带
    row_cnt = occ.sum(axis=1)
    col_cnt = occ.sum(axis=0)
    masked = np.where(occ, egrid, np.nan)
    row_mean = np.full(nu, np.nan)
    col_mean = np.full(npp, np.nan)
    gr = row_cnt > 20
    gc = col_cnt > 20
    row_mean[gr] = np.nanmean(masked[gr], axis=1)
    col_mean[gc] = np.nanmean(masked[:, gc], axis=0)

    def bands(profile, cell_mm, label, min_cells=8):
        valid = ~np.isnan(profile)
        v = profile[valid]
        m = np.median(v)
        mad = np.median(np.abs(v - m))
        thr = max(m + 2.5 * 1.4826 * mad, 1.0)
        flags = np.zeros(len(profile), bool)
        flags[valid] = profile[valid] > thr
        outb = []
        i = 0
        while i < len(flags):
            if flags[i]:
                j = i
                while j + 1 < len(flags) and flags[j + 1]:
                    j += 1
                if (j - i + 1) >= min_cells:
                    outb.append((i, j, float(np.nanmean(profile[i:j + 1]))))
                i = j + 1
            else:
                i += 1
        P(f"  {label}: thr={thr:.2f}mm -> {len(outb)} 条带")
        for (i0, i1, mg) in outb:
            P(f"    带: 轴向/位置 {i0 * cell_mm:.0f}..{i1 * cell_mm:.0f} mm, "
              f"宽 {(i1 - i0 + 1) * cell_mm:.0f} mm, 平均 {mg:+.2f} mm")
        return outb

    P("焊缝带(行剖面, 周向走向):")
    bands(row_mean, cell_u, "行", min_cells=8)
    P("轴向条带(列剖面):")
    bands(col_mean, cell_p * R_design, "列", min_cells=8)

    prof = smooth1d(interp_nan(row_mean), 3)
    save_line_plot(np.arange(len(prof)) * cell_u, prof,
                   os.path.join(out, "r3_raw_row_profile.png"),
                   f"raw cloud: mean radial residual per axial row (R fixed={R_design:.0f}mm)",
                   "axial position (mm)", "residual (mm)",
                   "(weld beads appear as double peaks)")
    prof_c = smooth1d(interp_nan(col_mean), 3)
    save_line_plot(np.arange(len(prof_c)) * cell_p * R_design, prof_c,
                   os.path.join(out, "r4_raw_col_profile.png"),
                   "raw cloud: mean radial residual per angle column",
                   "angle position (mm arc)", "residual (mm)", "")
    P("")

    # 标定点环带(高残差点连通域的圆状候选)
    thr_pt = max(med + 3 * sig, 1.2)
    high = occ & (egrid > thr_pt)
    lab, nl = label_components(high)
    P(f"点状凸起(>{thr_pt:.2f}mm)连通域: {nl} 个; 主要候选:")
    if nl:
        sizes = np.bincount(lab.ravel())[1:]
        order = np.argsort(sizes)[::-1]
        shown = 0
        for k in order:
            if sizes[k] < 40 or shown >= 10:
                continue
            ys, xs = np.where(lab == k + 1)
            du = (ys.max() - ys.min() + 1) * cell_u
            dp = (xs.max() - xs.min() + 1) * cell_p * R_design
            cy_ax = lo_t + ys.mean() * cell_u
            P(f"  候选{shown + 1}: {sizes[k]}格 ≈{du:.0f}x{dp:.0f}mm, "
              f"均值 +{float(np.nanmean(egrid[ys, xs])):.2f} mm, 轴向≈{cy_ax:.0f}mm")
            shown += 1
    P("")

    with open(os.path.join(out, "report_raw.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(rep))
    print("\n".join(rep))
    print(f"\n[OK] {out}")


if __name__ == "__main__":
    main()
