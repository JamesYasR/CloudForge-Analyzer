#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
diagnose_pothole_reference.py -- 凹塘测量"假结果"诊断

针对给定点云, 分别用不同的"设计半径"做固定半径圆柱拟合(等效程序里两阶段/三阶段
的寻优), 然后分析残差场, 回答三个问题:
  1) 残差里是否存在"系统性趋势"(碗形/倾斜) —— 即参考圆柱与真实面不匹配
  2) 自动阈值以下的点占多大比例 —— 即程序为何把"半个样本"当成凹塘
  3) 提取到的点群是否贴着补丁边界、椭圆是否溢出样本

用法:
    python3 diagnose_pothole_reference.py <pcd> <R1> <R2> ...
"""
import json
import math
import os
import sys
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_cylinder_patch import (read_pcd_xyz, ortho_frame, pca_axes,
                                    save_color_map, label_components)
from analyze_raw_with_prior import refine_axis_fixed_R, cyl_obj_fixed_R


def fit_ellipse(x, y):
    """Halir-Flusser 最小二乘椭圆拟合(与程序实现同)"""
    mx, my = x.mean(), y.mean()
    scale = math.sqrt(np.mean((x - mx) ** 2 + (y - my) ** 2))
    xx, yy = (x - mx) / scale, (y - my) / scale
    D1 = np.column_stack([xx * xx, xx * yy, yy * yy])
    D2 = np.column_stack([xx, yy, np.ones_like(xx)])
    S11 = D1.T @ D1; S12 = D1.T @ D2; S22 = D2.T @ D2
    M1i = np.array([[0, 0, 0.5], [0, -1, 0], [0.5, 0, 0]], float)
    S22i = np.linalg.inv(S22)
    M = M1i @ (S11 - S12 @ S22i @ S12.T)
    w, V = np.linalg.eig(M)
    for k in range(3):
        v1 = V[:, k].real
        disc = 4 * v1[0] * v1[2] - v1[1] ** 2
        if disc <= 1e-14:
            continue
        v1 = v1 / math.sqrt(disc)
        a2 = -S22i @ S12.T @ v1
        shift = (v1[0]*mx**2 + v1[1]*mx*my + v1[2]*my**2)/scale**2 - (a2[0]*mx + a2[1]*my)/scale
        for sg in (1.0, -1.0):
            A = sg * v1[0] / scale**2; B = sg * v1[1] / scale**2; C = sg * v1[2] / scale**2
            D_ = sg * ((-2*v1[0]*mx - v1[1]*my)/scale**2 + a2[0]/scale)
            E_ = sg * ((-2*v1[2]*my - v1[1]*mx)/scale**2 + a2[1]/scale)
            F_ = 1.0 + sg * shift
            if A < 0:
                A, B, C, D_, E_, F_ = -A, -B, -C, -D_, -E_, -F_
            if 4*A*C - B*B <= 1e-12:
                continue
            Mq = np.array([[2*A, B], [B, 2*C]])
            c0 = np.linalg.solve(Mq, np.array([-D_, -E_]))
            F0 = F_ + 0.5*(D_*c0[0] + E_*c0[1])
            if F0 >= -1e-12:
                continue
            ev, _ = np.linalg.eigh(Mq)
            if ev[0] <= 1e-12:
                continue
            s1 = math.sqrt(-F0/ev[0]); s2 = math.sqrt(-F0/ev[1])
            if s1 > 0 and s2 > 0:
                return c0, s1, s2, 0.5*math.atan2(B, A - C) + math.pi/2
    return None


def largest_grid_component(mask):
    """网格二值图的最大 8 连通域"""
    rows, cols = mask.shape
    lab = np.zeros((rows, cols), np.int32)
    best_id, best_size = 0, 0
    cur = 0
    for r in range(rows):
        for c in np.where(mask[r])[0]:
            if lab[r, c] == 0:
                cur += 1
                size = 0
                dq = deque([(r, c)])
                lab[r, c] = cur
                while dq:
                    y, x = dq.popleft()
                    size += 1
                    for dy in (-1, 0, 1):
                        for dx in (-1, 0, 1):
                            ny, nx = y + dy, x + dx
                            if 0 <= ny < rows and 0 <= nx < cols and mask[ny, nx] and lab[ny, nx] == 0:
                                lab[ny, nx] = cur
                                dq.append((ny, nx))
                if size > best_size:
                    best_size, best_id = size, cur
    return lab, best_id, best_size


def analyze(pts, c0, axis_hint, design_R, tag, out_dir, fixed_dir=None):
    rep = []
    P = rep.append
    P(f"================ 设计半径 R = {design_R:.1f} mm ================")

    if fixed_dir is not None:
        d = np.asarray(fixed_dir, float); d /= np.linalg.norm(d)
        P("  轴向: 采用 C++ 三阶段拟合结果(不再自行寻优)")
        sub = pts[::max(1, len(pts)//250000)]
        obj, a_c, b_c, t1, t2 = cyl_obj_fixed_R(sub, c0, d, design_R)
    else:
        sub = pts[::max(1, len(pts)//250000)]
        d, _ = refine_axis_fixed_R(sub, c0, axis_hint, design_R,
                                   schedule=((1.0, 0.25), (0.3, 0.1)))
        obj, a_c, b_c, t1, t2 = cyl_obj_fixed_R(sub, c0, d, design_R)

    v = pts - c0
    a_ax = v @ d
    u = v @ t1; w = v @ t2
    rho = np.hypot(u - a_c, w - b_c)
    e = rho - design_R
    phi = np.arctan2(w - b_c, u - a_c)
    s_arc = design_R * phi

    med = float(np.median(e))
    sig = 1.4826 * float(np.median(np.abs(e - med)))
    thr = max(3.0 * sig, 0.8)
    below = e < -thr
    P(f"  截尾RMS(子采样) = {obj:.3f} mm; 全量残差 median={med:+.3f} 稳健sigma={sig:.3f} mm")
    P(f"  残差范围 p0.1={np.percentile(e,0.1):+.2f} p99.9={np.percentile(e,99.9):+.2f} mm")
    P(f"  自动阈值 thr = {thr:.3f} mm -> 低于阈值的点占比 = {below.mean()*100:.2f}%")
    for t in (0.5, 1.0, 2.0, 3.0, 4.0):
        P(f"     若手动阈值取 {t:.1f} mm -> 低于阈值(即被判为凹塘)的点占比 = "
          f"{(e < -t).mean()*100:.2f}%")

    # 系统性趋势: 低阶多项式拟合残差场
    idx = np.arange(0, len(pts), max(1, len(pts)//200000))
    A = np.column_stack([np.ones(len(idx)), a_ax[idx], s_arc[idx],
                         a_ax[idx]**2, s_arc[idx]**2, a_ax[idx]*s_arc[idx]])
    coef, *_ = np.linalg.lstsq(A, e[idx], rcond=None)
    trend = A @ coef
    P(f"  残差场低阶趋势(平面+二次)幅度 p2p = {trend.max()-trend.min():.3f} mm"
      f"  vs  去趋势后稳健sigma = {1.4826*np.median(np.abs(e[idx]-trend-np.median(e[idx]-trend))):.3f} mm")
    ratio = (trend.max()-trend.min()) / max(1e-9, 1.4826*np.median(np.abs(e[idx]-trend-np.median(e[idx]-trend))))
    P(f"  趋势/局部起伏 比值 = {ratio:.1f}  (>3 说明残差被系统性偏差主导, 不是局部凹塘)")

    # 网格化候选点 -> 最大连通域 -> 是否贴边 / 椭圆是否溢出
    a_lo, a_hi = np.percentile(a_ax, [0.1, 99.9])
    s_lo, s_hi = np.percentile(s_arc, [0.1, 99.9])
    cell = max((a_hi - a_lo) / 300.0, (s_hi - s_lo) / 300.0)
    na = int((a_hi - a_lo) / cell) + 1
    ns = int((s_hi - s_lo) / cell) + 1
    ia = np.clip(((a_ax - a_lo) / cell).astype(int), 0, na - 1)
    isx = np.clip(((s_arc - s_lo) / cell).astype(int), 0, ns - 1)
    occ_all = np.zeros((na, ns), bool)
    occ_all[ia, isx] = True
    occ_cand = np.zeros((na, ns), bool)
    occ_cand[ia[below], isx[below]] = True

    lab, best_id, best_size = largest_grid_component(occ_cand)
    main = lab == best_id
    P(f"  候选点群最大连通域 = {best_size} 格 / 补丁占据 {occ_all.sum()} 格"
      f" = 补丁面积的 {best_size/max(1,occ_all.sum())*100:.1f}%")
    rows = np.where(main.any(axis=1))[0]
    cols = np.where(main.any(axis=0))[0]
    touch = (rows[0] <= 1 or rows[-1] >= na - 2 or cols[0] <= 1 or cols[-1] >= ns - 2)
    P(f"  该点群是否贴到补丁边界: {'是 (说明不是内部凹塘, 而是参考面误差区域)' if touch else '否'}")

    b = np.zeros_like(main)
    b[1:-1, 1:-1] = main[1:-1, 1:-1] & (~main[:-2, 1:-1] | ~main[2:, 1:-1] |
                                        ~main[1:-1, :-2] | ~main[1:-1, 2:])
    ys, xs = np.where(b)
    if len(ys) >= 16:
        cx = a_lo + (ys + 0.5) * cell
        cy = s_lo + (xs + 0.5) * cell
        r = fit_ellipse(cx, cy)
        if r:
            (ex, ey), s1, s2, ang = r
            P(f"  点群轮廓椭圆: 长轴={2*s1:.1f} 短轴={2*s2:.1f} mm, 中心(轴向{ex:.1f}, 周向{ey:.1f})")
            ea = [ex - s1, ex + s1]
            P(f"  椭圆轴向跨度 [{ea[0]:.1f}, {ea[1]:.1f}] vs 补丁轴向 [{a_lo:.1f}, {a_hi:.1f}]"
              f" -> {'溢出样本!' if ea[0] < a_lo or ea[1] > a_hi else '在样本内'}")
        else:
            P("  椭圆拟合失败")
    # 输出残差图
    egrid = np.full((na, ns), np.nan)
    cnt = np.zeros((na, ns))
    np.add.at(egrid, (ia, isx), e)
    np.add.at(cnt, (ia, isx), 1)
    with np.errstate(invalid='ignore'):
        egrid = np.where(cnt > 0, egrid / np.maximum(cnt, 1), np.nan)
    vmax = max(3 * sig, 0.5)
    save_color_map(np.where(occ_all, np.nan_to_num(egrid, nan=0.0), 0.0),
                   os.path.join(out_dir, f"diag_residual_R{int(design_R)}.png"), vmax, scale=2)
    P(f"  残差图已保存: diag_residual_R{int(design_R)}.png (色标 ±{vmax:.2f} mm)")
    return rep, (d, a_c, b_c, sig, thr, below.mean(), ratio, best_size / max(1, occ_all.sum()))


def main():
    cloud = sys.argv[1] if len(sys.argv) > 1 else \
        "/media/jamesyasr/Shared/大创/点云/test2/1_cld.pcd"
    args = sys.argv[2:]
    jobs = []
    i = 0
    while i < len(args):
        R = float(args[i]); i += 1
        dirv = None
        if i < len(args):
            try:
                dirv = [float(v) for v in args[i].split(',')]
                i += 1
            except ValueError:
                dirv = None
        jobs.append((R, dirv))
    if not jobs:
        jobs = [(1940.0, None), (1260.11, None)]
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out_diag")
    os.makedirs(out_dir, exist_ok=True)

    pts, _ = read_pcd_xyz(cloud)
    pts = pts[np.isfinite(pts).all(axis=1)]
    c0 = pts.mean(axis=0)
    _, vec, ev = pca_axes(pts[::max(1, len(pts)//300000)])
    print(f"点云: {cloud}  N={len(pts)}")
    print(f"PCA 主轴(std): {np.sqrt(ev).round(2)}")

    axis_hint = vec[:, 0]
    for R, dirv in jobs:
        rep, summ = analyze(pts, c0, axis_hint, R, f"R{R:.0f}", out_dir, fixed_dir=dirv)
        print("\n".join(rep)); print()
        axis_hint = summ[0]   # 用上一个结果作下一个初值
    print(f"输出目录: {out_dir}")


if __name__ == "__main__":
    main()
