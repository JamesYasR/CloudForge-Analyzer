#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ruleA_reference.py -- 口径A(local=0, 改造前的单坑路径)的独立 Python 复现

目的: 给"口径A 数值与改造前一致"提供**独立证据**。
本脚本用 numpy/python 独立复现改造前的算法(不是调用 C++ 代码):
  1) e = ||perp(v-p0, d)|| - R                        (点到理想柱面带符号距离)
  2) 形面趋势: 展开域二阶最小二乘 + 两轮 3-sigma 截尾, 自动模式(比值>2 则扣除)
  3) 距离阈值: 手动 or max(3*robust_sigma_work, 0.8)
  4) 候选点: e_work < -thr
  5) 欧氏聚类(栅格 3mm 半径连通, 与 PCL EuclideanClusterExtraction 等价语义)
  6) 取最大簇 -> 展开域栅格轮廓 -> 最小二乘椭圆 (Halir-Flusser)
  7) 四道闸门

用法:
  python3 ruleA_reference.py <pcd> <R> <axis_file> [thr] [trend_mode] [area_pct]
axis_file: 6 个数 px py pz dx dy dz(与 test_cyl_pothole 的 axis= 缓存同格式)
"""
import math
import sys
from collections import deque

import numpy as np

KR = 1.4826


def read_pcd_xyz(path):
    with open(path, "rb") as f:
        hdr = {}
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError("PCD 头损坏")
            if line.startswith(b"#"):
                continue
            parts = line.split()
            if not parts:
                continue
            hdr[parts[0].decode()] = parts[1:]
            if parts[0] == b"DATA":
                break
        fields = [x.decode() for x in hdr["FIELDS"]]
        sizes = [int(x) for x in hdr["SIZE"]]
        types = [x.decode() for x in hdr["TYPE"]]
        counts = [int(x) for x in hdr["COUNT"]]
        n = int(hdr["POINTS"][0])
        assert fields == ["x", "y", "z"] and sizes == [4, 4, 4] and types == ["F", "F", "F"]
        raw = f.read(n * 12)
        pts = np.frombuffer(raw, dtype=np.float32).reshape(-1, 3).astype(np.float64)
    return pts


def robust_sigma(v):
    med = np.median(v)
    return KR * np.median(np.abs(v - med)), med


def main():
    pcd, R = sys.argv[1], float(sys.argv[2])
    ax = np.loadtxt(sys.argv[3])
    p0, d = ax[:3], ax[3:]
    d = d / np.linalg.norm(d)
    thr_manual = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    trend_mode = int(sys.argv[5]) if len(sys.argv) > 5 else 0
    area_pct = float(sys.argv[6]) if len(sys.argv) > 6 else 20.0

    pts = read_pcd_xyz(pcd)
    n = len(pts)
    rel = pts - p0
    a = rel @ d
    rho = np.linalg.norm(rel - np.outer(a, d), axis=1)
    e = rho - R
    sigA, _ = robust_sigma(e)
    print(f"N={n} R={R} axis_p=({p0[0]:.6f},{p0[1]:.6f},{p0[2]:.6f}) "
          f"axis_d=({d[0]:.9f},{d[1]:.9f},{d[2]:.9f})")
    print(f"[口径A] 稳健 sigma = {sigA:.9f}, 最大凹深 = {-e.min():.9f}, "
          f"最深点 = ({pts[e.argmin()][0]:.6f},{pts[e.argmin()][1]:.6f},{pts[e.argmin()][2]:.6f})")

    # 展开坐标(与 C++ 同一 t1/t2 构造: t1 = X - d*(X.d) 归一化, t2 = d x t1)
    x0 = np.array([1.0, 0.0, 0.0]) if abs(d[0]) <= 0.9 else np.array([0.0, 1.0, 0.0])
    t1 = x0 - d * (x0 @ d); t1 /= np.linalg.norm(t1)
    t2 = np.cross(d, t1)
    s = R * np.arctan2(rel @ t2, rel @ t1)
    amin, amax = a.min(), a.max()
    smin, smax = s.min(), s.max()
    patch_area = max(1e-6, (amax - amin) * (smax - smin))

    # 形面趋势: 二阶 + 两轮截尾(与 C++ 同构)
    def basis(aa, ss):
        return np.stack([np.ones_like(aa), aa, ss, aa * aa, ss * ss, aa * ss], axis=1)
    B = basis(a, s)
    stride = max(1, n // 200000)
    idx = np.arange(0, n, stride)
    keep = np.ones(len(idx), bool)
    coef = np.zeros(6)
    for _ in range(2):
        Bi = B[idx][keep]
        sol, *_ = np.linalg.lstsq(Bi, e[idx][keep], rcond=None)
        coef = sol
        dev = e[idx] - B[idx] @ coef
        med = np.median(dev)
        mad = np.median(np.abs(dev - med))
        lim = 3.0 * KR * max(mad, 1e-9)
        keep = np.abs(dev - med) <= lim
    tr = B @ coef
    tmin, tmax = tr[idx].min(), tr[idx].max()
    trend_p2p = tmax - tmin
    localres = e[idx] - (B[idx] @ coef)
    sigW, _ = robust_sigma(localres)
    ratio = trend_p2p / max(sigW, 1e-6)
    removed = (trend_mode == 2) or (trend_mode == 0 and ratio > 2.0)
    ew = e - tr if removed else e.copy()
    if removed:
        sigW, _ = robust_sigma(ew[idx])
        print(f"[趋势] p2p={trend_p2p:.9f} 局部sigma={sigW:.9f} 比值={ratio:.6f} -> 已扣除")
    else:
        print(f"[趋势] p2p={trend_p2p:.9f} 局部sigma={sigW:.9f} 比值={ratio:.6f} -> 未扣除")

    thr = thr_manual if thr_manual > 0 else max(3.0 * sigW, 0.8)
    print(f"[口径A] 距离阈值 = {thr:.9f} ({'手动' if thr_manual > 0 else '自动'})")

    cand = np.where(ew < -thr)[0]
    print(f"[口径A] 候选点数 = {len(cand)}")
    if len(cand) < 200:
        print("候选点不足 -> 未检出")
        return

    # 欧氏聚类: 3mm 球形连通(网格半径搜索, 与 PCL 语义一致: dist <= tol 即连通)
    tol = 3.0
    P = pts[cand]
    inv = 1.0 / tol
    keys = np.floor(P * inv).astype(np.int64)
    grid = {}
    for i, k in enumerate(map(tuple, keys)):
        grid.setdefault(k, []).append(i)
    lbl = np.full(len(P), -1, np.int32)
    clusters = []
    for st in range(len(P)):
        if lbl[st] >= 0:
            continue
        cid = len(clusters)
        q = deque([st]); lbl[st] = cid; members = []
        while q:
            i = q.popleft(); members.append(i)
            kx, ky, kz = keys[i]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        for j in grid.get((kx + dx, ky + dy, kz + dz), ()):
                            if lbl[j] < 0 and np.linalg.norm(P[j] - P[i]) <= tol:
                                lbl[j] = cid; q.append(j)
        clusters.append(np.array(members))
    sizes = sorted((len(c) for c in clusters), reverse=True)
    print(f"[口径A] 聚类容差 = 3.0 mm, 簇数 = {len(clusters)}, 各簇点数(降序) = {sizes[:8]}")
    best = max(clusters, key=len)
    if len(best) < 200:
        print("最大簇不足最小点数 -> 未检出")
        return
    orig = cand[best]
    Q = pts[orig]
    ea, es = a[orig], s[orig]
    qamin, qamax = ea.min(), ea.max()
    qsmin, qsmax = es.min(), es.max()
    print(f"[口径A] 点群 = {len(orig)} 点, 平均深度(到柱面) = {(-e[orig]).mean():.9f}")
    ap_mean = -e[orig].mean()
    if not (qamax - qamin >= 4.0 and qsmax - qsmin >= 4.0):
        print("[口径A] 点群展开尺寸过小 -> 椭圆拟合失败")
        return

    # 展开域栅格轮廓(与 C++ fitEllipseOnContour 同构)
    da, ds = qamax - qamin, qsmax - qsmin
    spacing = math.sqrt(da * ds / len(orig))
    cell = max(1.0, 2.2 * spacing)
    na = max(4, int(math.ceil(da / cell)) + 1)
    ns = max(4, int(math.ceil(ds / cell)) + 1)
    mask = np.zeros((na, ns), bool)
    ia = np.clip(((ea - qamin) / cell).astype(int), 0, na - 1)
    isx = np.clip(((es - qsmin) / cell).astype(int), 0, ns - 1)
    mask[ia, isx] = True
    # 最大连通域(8 连通)
    comp = np.zeros((na, ns), np.int32); cid = 0; bestc = 0; bestsz = 0
    for r0 in range(na):
        for c0 in range(ns):
            if not mask[r0, c0] or comp[r0, c0]:
                continue
            cid += 1; sz = 0; q = deque([(r0, c0)]); comp[r0, c0] = cid
            while q:
                y, x = q.popleft(); sz += 1
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < na and 0 <= nx < ns and mask[ny, nx] and not comp[ny, nx]:
                            comp[ny, nx] = cid; q.append((ny, nx))
            if sz > bestsz:
                bestsz, bestc = sz, cid
    contour = []
    for rr in range(na):
        for cc in range(ns):
            if comp[rr, cc] != bestc:
                continue
            edge = (rr == 0 or rr == na - 1 or cc == 0 or cc == ns - 1
                    or not mask[rr - 1, cc] or not mask[rr + 1, cc]
                    or not mask[rr, cc - 1] or not mask[rr, cc + 1])
            if edge:
                contour.append((qamin + (rr + 0.5) * cell, qsmin + (cc + 0.5) * cell))
    contour = np.array(contour)
    print(f"[口径A] 栅格边界点数 = {len(contour)} (栅格 {na}x{ns}, cell={cell:.9f}mm)")
    if len(contour) < 16:
        print("[口径A] 边界点过少 -> 椭圆拟合失败")
        return

    # Halir-Flusser 椭圆拟合(与 C++ 同一算法)
    x = contour[:, 0]; y = contour[:, 1]
    mu = np.array([x.mean(), y.mean()])
    sc = math.sqrt(((np.stack([x, y], 1) - mu) ** 2).sum(1).mean())
    X = (x - mu[0]) / sc; Y = (y - mu[1]) / sc
    D1 = np.stack([X * X, X * Y, Y * Y], 1)
    D2 = np.stack([X, Y, np.ones_like(X)], 1)
    S1, S2, S3 = D1.T @ D1, D1.T @ D2, D2.T @ D2
    C1 = np.array([[0, 0, 0.5], [0, -1, 0], [0.5, 0, 0]])
    M = C1 @ (S1 - S2 @ np.linalg.inv(S3) @ S2.T)
    w, v = np.linalg.eig(M)
    ok = False
    for k in range(3):
        v1 = np.real(v[:, k])
        disc = 4 * v1[0] * v1[2] - v1[1] ** 2
        if disc <= 1e-14:
            continue
        v1 = v1 / math.sqrt(disc)
        a2 = -np.linalg.inv(S3) @ S2.T @ v1
        dx, dy, aa = mu[0], mu[1], sc
        shift = (v1[0] * dx * dx + v1[1] * dx * dy + v1[2] * dy * dy) / (aa * aa) \
            - (a2[0] * dx + a2[1] * dy) / aa
        for sg in (1.0, -1.0):
            A = sg * v1[0] / (aa * aa); B = sg * v1[1] / (aa * aa); C = sg * v1[2] / (aa * aa)
            D = sg * ((-2 * v1[0] * dx - v1[1] * dy) / (aa * aa) + a2[0] / aa)
            E = sg * ((-2 * v1[2] * dy - v1[1] * dx) / (aa * aa) + a2[1] / aa)
            F = 1.0 + sg * shift
            if A < 0:
                A, B, C, D, E, F = -A, -B, -C, -D, -E, -F
            if 4 * A * C - B * B <= 1e-12:
                continue
            Mq = np.array([[2 * A, B], [B, 2 * C]])
            c0 = np.linalg.solve(Mq, np.array([-D, -E]))
            F0 = F + 0.5 * (D * c0[0] + E * c0[1])
            if F0 >= -1e-12:
                continue
            l1, l2 = np.linalg.eigvalsh(Mq)
            if l1 <= 1e-12 or l2 <= 1e-12:
                continue
            sm1 = math.sqrt(-F0 / l1); sm2 = math.sqrt(-F0 / l2)
            ang = 0.5 * math.atan2(B, A - C) + math.pi / 2
            print(f"[口径A] 椭圆: 长轴 {2*sm1:.9f} mm, 短轴 {2*sm2:.9f} mm, "
                  f"中心(展开域 a,s) = ({c0[0]:.6f}, {c0[1]:.6f}), 长宽比 {sm1/sm2:.6f}")
            area_frac = bestsz * cell * cell / patch_area
            tol_a = max(2.0 * cell, 0.02 * (amax - amin))
            tol_s = max(2.0 * cell, 0.02 * (smax - smin))
            touch = ((qamin - amin < tol_a) or (amax - qamax < tol_a)
                     or (qsmin - smin < tol_s) or (smax - qsmax < tol_s))
            print(f"[口径A] 点群面积占比 = {area_frac*100:.6f}%, 贴补丁边界 = "
                  f"{'是' if touch else '否'}")
            ok = True
            break
        if ok:
            break
    if not ok:
        print("[口径A] 椭圆拟合失败")
    print(f"[KEY-PY] valid={int(ok)} pit_points={len(orig)} cluster_count={len(clusters)} "
          f"mean_depth={ap_mean:.9f}")


if __name__ == "__main__":
    main()
