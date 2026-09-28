#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_cylinder_patch.py (v2) -- 局部圆柱点云(火箭贮箱筒段小视角 patch)特征分析

用法:
    python3 analyze_cylinder_patch.py <input.pcd> [out_dir]

依赖: numpy, Pillow (无 scipy / matplotlib / open3d)

v2 相对 v1:
  * binary_compressed 按 PCL 的 SoA(planar) 字段重排布局解析
  * 轴向搜索: PCA1 / PCA2 双起点 + 重心化多轮粗到精(解决近方形 patch 的 90° 歧义)
  * 稳健半径: median(rho) + 固定半径圆心精修
  * 多尺度占据分析(空白区随尺度的稳定性)
  * 焊缝带: 行/列平均残差剖面 + 稳健阈值(与程序第三阶段思路一致)
  * 标定点: 圆形空洞 + 环带凸起统计
"""
import json
import math
import os
import sys
from collections import deque

import numpy as np
from PIL import Image, ImageDraw

# ---------------------------------------------------------------- PCD 读取

def lzf_decompress(data: bytes, expected: int) -> bytes:
    """PCL/LZF 解压 (liblzf 格式, 与 PCL pcd_io binary_compressed 一致)."""
    out = bytearray()
    ip = 0
    n = len(data)
    while ip < n:
        ctrl = data[ip]
        ip += 1
        if ctrl < 32:  # literal run: ctrl+1 字节原样拷贝
            lit = ctrl + 1
            out += data[ip:ip + lit]
            ip += lit
        else:          # back reference
            length = ctrl >> 5
            if length == 7:
                length += data[ip]
                ip += 1
            ref = len(out) - ((ctrl & 0x1F) << 8) - data[ip] - 1
            ip += 1
            length += 2
            if ref < 0:
                raise ValueError("lzf: 非法回引 offset")
            if ref + length <= len(out):
                out += out[ref:ref + length]
            else:      # 重叠区只能逐字节
                for k in range(length):
                    out.append(out[ref + k])
    if len(out) != expected:
        raise ValueError(f"lzf: 解压后 {len(out)}B != 期望 {expected}B")
    return bytes(out)


_TYPES_NP = {("F", 4): "f4", ("F", 8): "f8", ("U", 4): "u4", ("U", 2): "u2",
             ("U", 1): "u1", ("I", 4): "i4", ("I", 2): "i2", ("I", 1): "i1"}


def read_pcd_xyz(path: str):
    """读取 PCD(支持 binary / binary_compressed / ascii), 返回 (N,3) float32."""
    with open(path, "rb") as f:
        blob = f.read()
    marks = [(blob.find(b"DATA binary_compressed\n"), b"binary_compressed"),
             (blob.find(b"DATA binary\n"), b"binary"),
             (blob.find(b"DATA ascii\n"), b"ascii")]
    cand = [(p, m) for p, m in marks if p >= 0]
    if not cand:
        raise ValueError("找不到 DATA 段")
    pos, kind = min(cand, key=lambda t: t[0])
    header = blob[:pos].decode("ascii", "replace").splitlines()
    fields, sizes, types, counts, npoints = [], [], [], [], None
    for line in header:
        parts = line.split()
        if not parts:
            continue
        key = parts[0].upper()
        if key == "FIELDS":
            fields = parts[1:]
        elif key == "SIZE":
            sizes = [int(v) for v in parts[1:]]
        elif key == "TYPE":
            types = parts[1:]
        elif key == "COUNT":
            counts = [int(v) for v in parts[1:]]
        elif key == "POINTS":
            npoints = int(parts[1])
    if not counts or len(counts) != len(fields):
        counts = [1] * len(fields)
    nvals = sum(counts)

    def get_planar(raw: bytes):
        """PCL binary_compressed 解压后是 SoA: 按字段元素连续存放."""
        planar, off = {}, 0
        for f_name, sz, tp, cnt in zip(fields, sizes, types, counts):
            base = _TYPES_NP[(tp.upper(), sz)]
            for k in range(cnt):
                planar[(f_name, k)] = np.frombuffer(raw, dtype="<" + base,
                                                    count=npoints, offset=off)
                off += npoints * sz
        return planar

    body = blob[pos + len(b"DATA ") + len(kind) + 1:]
    if kind == b"binary_compressed":
        comp = int(np.frombuffer(body[:4], dtype="<u4")[0])
        unc = int(np.frombuffer(body[4:8], dtype="<u4")[0])
        planar = get_planar(lzf_decompress(bytes(body[8:8 + comp]), unc))
    elif kind == b"binary":
        point_step = sum(s * c for s, c in zip(sizes, counts))
        raw = body[:npoints * point_step]
        # 交错(AoS)布局: 每个点依次存放各字段
        dt = np.dtype([(f"f{i}", "<" + _TYPES_NP[(tp.upper(), sz)], (cnt,))
                       for i, (sz, tp, cnt) in enumerate(zip(sizes, types, counts))])
        rec = np.frombuffer(raw, dtype=dt, count=npoints)
        planar = {}
        for i, (f_name, sz, tp, cnt) in enumerate(zip(fields, sizes, types, counts)):
            col = rec[f"f{i}"]
            for k in range(cnt):
                planar[(f_name, k)] = np.asarray(col).reshape(npoints) if cnt == 1 else col[:, k]
    else:
        arr = np.loadtxt(body.decode("ascii").splitlines()[:npoints],
                         dtype=np.float32, usecols=(0, 1, 2))
        return arr.astype(np.float32), npoints
    xyz = np.stack([planar[("x", 0)], planar[("y", 0)], planar[("z", 0)]], axis=1)
    return xyz.astype(np.float32), npoints


# ---------------------------------------------------------------- 基础几何

def pca_axes(pts: np.ndarray):
    """返回 (中心, 特征值降序的 3x3 特征向量, 特征值降序)."""
    c = pts.mean(axis=0)
    q = (pts - c).astype(np.float64)
    cov = (q.T @ q) / max(1, len(q) - 1)
    w, v = np.linalg.eigh(cov)
    order = np.argsort(w)[::-1]
    return c, v[:, order], w[order]


def ortho_frame(d: np.ndarray):
    """单位向量 d 的正交框架 (d, t1, t2)."""
    d = d / np.linalg.norm(d)
    a = np.array([1.0, 0.0, 0.0])
    if abs(d @ a) > 0.9:
        a = np.array([0.0, 1.0, 0.0])
    t1 = a - d * (a @ d)
    t1 /= np.linalg.norm(t1)
    t2 = np.cross(d, t1)
    return d, t1, t2


def circle_fit_kasa(u: np.ndarray, w: np.ndarray):
    A = np.stack([2 * u, 2 * w, np.ones_like(u)], axis=1).astype(np.float64)
    b = (u.astype(np.float64) ** 2 + w.astype(np.float64) ** 2)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    a, bb, k = sol
    return float(a), float(bb), math.sqrt(max(k + a * a + bb * bb, 1e-12))


def circle_fit_gn(u, w, a, b, r, iters=8, fit_r=True):
    """几何残差 Gauss-Newton, 带半步阻尼与 r>0 守卫."""
    u = u.astype(np.float64)
    w = w.astype(np.float64)
    base_rms = None
    for _ in range(iters):
        du, dw = u - a, w - b
        rho = np.sqrt(du * du + dw * dw) + 1e-12
        res = rho - r
        base_rms = float(np.sqrt(np.mean(res ** 2)))
        j0, j1 = -du / rho, -dw / rho
        J = (np.stack([j0, j1, -np.ones_like(j0)], axis=1) if fit_r
             else np.stack([j0, j1], axis=1))
        g = J.T @ res
        H = J.T @ J + 1e-9 * np.eye(J.shape[1])
        try:
            step = np.linalg.solve(H, g)
        except np.linalg.LinAlgError:
            break
        lam = 1.0
        ok = False
        for _ in range(4):
            a2, b2 = a + lam * step[0], b + lam * step[1]
            r2 = r + (lam * step[2] if fit_r else 0.0)
            if r2 > 1e-3:
                rr = np.hypot(u - a2, w - b2) - r2
                if float(np.sqrt(np.mean(rr ** 2))) <= base_rms + 1e-12:
                    ok = True
                    break
            lam *= 0.5
        if not ok:
            break
        a, b, r = a2, b2, r2
        if np.hypot(step[0], step[1]) * lam < 1e-6 and (not fit_r or abs(step[2]) * lam < 1e-6):
            break
    return a, b, max(r, 1e-3)


def fit_cyl_to_points(pts, center, d, iters_gn=8):
    """给定轴: 投影->Kasa->GN自由半径->median半径->GN固定半径精修圆心.
    返回 dict(a,b,R,obj,obj_med)."""
    _, t1, t2 = ortho_frame(d)
    v = pts - center
    u = v @ t1
    w = v @ t2
    a0, b0, r0 = circle_fit_kasa(u, w)
    a, b, r1 = circle_fit_gn(u, w, a0, b0, r0, iters=iters_gn, fit_r=True)
    rho = np.hypot(u - a, w - b)
    R = float(np.median(rho))
    a, b, _ = circle_fit_gn(u, w, a, b, R, iters=iters_gn, fit_r=False)
    rho = np.hypot(u - a, w - b)
    e = rho - R
    k = max(1, int(0.95 * len(e)))
    part = np.partition(np.abs(e), k - 1)[:k]
    obj = float(np.sqrt(np.mean(part ** 2)))
    return dict(a=a, b=b, R=R, t1=t1, t2=t2, obj=obj,
                e=e, rho=rho, u=u, w=w)


def axis_with_tilt(d0, alpha, beta):
    d0 = d0 / np.linalg.norm(d0)
    _, t1, t2 = ortho_frame(d0)
    return d0 + math.tan(alpha) * t1 + math.tan(beta) * t2


def refine_axis(sub, center, d0, schedule=((6.0, 1.0), (2.0, 0.25), (0.6, 0.1), (0.2, 0.04))):
    """重心化多轮粗到精轴搜索. 返回 (d_best, obj_best, total_tilt_deg, trace)."""
    d = d0 / np.linalg.norm(d0)
    trace = []
    for rng_deg, step_deg in schedule:
        for _ in range(4):  # 同尺度内重心化, 最多 4 轮
            _, t1, t2 = ortho_frame(d)
            base_obj = fit_cyl_to_points(sub, center, d)["obj"]
            grid = np.arange(-rng_deg, rng_deg + 1e-9, step_deg)
            best = (base_obj, 0.0, 0.0)
            for al_d in grid:
                for be_d in grid:
                    if al_d == 0.0 and be_d == 0.0:
                        continue
                    al, be = math.radians(al_d), math.radians(be_d)
                    dd = d + math.tan(al) * t1 + math.tan(be) * t2
                    o = fit_cyl_to_points(sub, center, dd)["obj"]
                    if o < best[0]:
                        best = (o, al, be)
            d = d + math.tan(best[1]) * t1 + math.tan(best[2]) * t2
            d /= np.linalg.norm(d)
            trace.append((round(math.degrees(best[1]), 3),
                          round(math.degrees(best[2]), 3),
                          round(best[0], 4)))
            if best[1] == 0.0 and best[2] == 0.0:
                break
    obj = fit_cyl_to_points(sub, center, d)["obj"]
    return d, obj, trace


def circular_span(phi):
    ph = np.sort(phi.astype(np.float64))
    if len(ph) < 2:
        return 0.0, 0.0
    gaps = np.diff(ph)
    wrap_gap = (ph[0] + 2 * math.pi) - ph[-1]
    max_gap = max(float(gaps.max()) if len(gaps) else 0.0, wrap_gap)
    span = 2 * math.pi - max_gap
    start = ph[-1] if max_gap == wrap_gap else ph[int(np.argmax(gaps)) + 1]
    return span, start + span / 2


# ---------------------------------------------------------------- 位图输出

def save_gray_mask(mask, path, scale=1):
    im = Image.fromarray(np.where(mask, 0, 255).astype(np.uint8), mode="L")
    if scale > 1:
        im = im.resize((im.width * scale, im.height * scale), Image.NEAREST)
    im.save(path)


def save_color_map(grid, path, vmax, scale=2):
    t = np.clip(grid / max(vmax, 1e-9), -1.0, 1.0)
    r = np.where(t >= 0, 255, 255 - 255 * (-t)).astype(np.uint8)
    g = (255 - 255 * np.abs(t)).astype(np.uint8)
    b = np.where(t >= 0, 255 - 255 * t, 255).astype(np.uint8)
    rgb = np.stack([r, g, b], axis=-1)
    im = Image.fromarray(rgb, mode="RGB")
    im = im.resize((im.width * scale, im.height * scale), Image.NEAREST)
    im.save(path)


def save_heatmap(grid, path, title, xlab, ylab):
    g = grid.astype(np.float64)
    lo, hi = float(g.min()), float(g.max())
    t = (g - lo) / max(hi - lo, 1e-12)
    stops = [(68, 1, 84), (59, 158, 132), (222, 195, 45), (253, 231, 37)]
    tt = t * (len(stops) - 1)
    i0 = np.clip(np.floor(tt).astype(int), 0, len(stops) - 2)
    frac = tt - i0
    rgb = np.zeros((*t.shape, 3), np.uint8)
    for c in range(3):
        a0 = np.array([s[c] for s in stops], dtype=np.float64)
        rgb[..., c] = np.clip(a0[i0] * (1 - frac) + a0[i0 + 1] * frac, 0, 255).astype(np.uint8)
    cell = 24
    h, w = g.shape
    im = Image.fromarray(rgb, "RGB").resize((w * cell, h * cell), Image.NEAREST)
    dr = ImageDraw.Draw(im)
    dr.rectangle([0, 0, im.width - 1, im.height - 1], outline=(0, 0, 0))
    dr.text((8, 6), f"{title}  min={lo:.4f} max={hi:.4f} (mm)", fill=(0, 0, 0))
    dr.text((8, im.height - 40), f"x: {xlab}", fill=(0, 0, 0))
    dr.text((8, im.height - 24), f"y: {ylab}", fill=(0, 0, 0))
    im.save(path)


def save_line_plot(xs, ys, path, title, xlab, ylab, fname_note=""):
    im = Image.new("RGB", (900, 380), (255, 255, 255))
    dr = ImageDraw.Draw(im)
    x0, y0 = 80, 40
    pw, ph = 900 - x0 - 30, 380 - y0 - 70
    dr.rectangle([x0, y0, x0 + pw, y0 + ph], outline=(60, 60, 60))
    xs = np.asarray(xs, float)
    ys = np.asarray(ys, float)
    px = x0 + pw * (xs - xs.min()) / max(xs.max() - xs.min(), 1e-12)
    py = y0 + ph * (1 - (ys - ys.min()) / max(ys.max() - ys.min(), 1e-12))
    dr.line(list(zip(px, py)), fill=(180, 40, 40), width=3)
    for x, y in zip(px, py):
        dr.ellipse([x - 3, y - 3, x + 3, y + 3], fill=(180, 40, 40))
    dr.text((x0, 8), title, fill=(0, 0, 0))
    dr.text((x0, 24), f"{xlab} vs {ylab}  {fname_note}", fill=(90, 90, 90))
    dr.text((x0 + pw - 130, y0 + ph + 8), f"{xs.min():.2f} .. {xs.max():.2f}", fill=(0, 0, 0))
    dr.text((12, y0 - 22), f"{ys.max():.3f}", fill=(0, 0, 0))
    dr.text((12, y0 + ph - 4), f"{ys.min():.3f}", fill=(0, 0, 0))
    im.save(path)


def save_histogram(e, path, title, clip=4.0):
    ec = np.clip(e, -clip, clip)
    bins = np.linspace(-clip, clip, 161)
    hist, _ = np.histogram(ec, bins=bins)
    bw, bh = 900, 360
    im = Image.new("RGB", (bw, bh), (255, 255, 255))
    dr = ImageDraw.Draw(im)
    peak = hist.max()
    x0, y0 = 60, 30
    plot_w, plot_h = bw - x0 - 20, bh - y0 - 70
    dr.rectangle([x0, y0, x0 + plot_w, y0 + plot_h], outline=(60, 60, 60))
    for i, hgt in enumerate(hist):
        bx0 = x0 + plot_w * i / len(hist)
        bx1 = x0 + plot_w * (i + 1) / len(hist)
        by1 = y0 + plot_h
        by0 = y0 + plot_h * (1 - hgt / peak)
        dr.rectangle([bx0, by0, bx1, by1], fill=(70, 110, 200))
    zero_x = x0 + plot_w * (0 - (-clip)) / (2 * clip)
    dr.line([zero_x, y0, zero_x, y0 + plot_h], fill=(200, 40, 40), width=2)
    dr.text((x0, bh - 44), f"{title}", fill=(0, 0, 0))
    dr.text((x0, bh - 28), f"x: radial residual mm (-{clip}..+{clip})   N={len(e)} peak={peak}",
            fill=(0, 0, 0))
    im.save(path)


def label_components(mask):
    h, w = mask.shape
    labels = np.zeros((h, w), np.int32)
    cur = 0
    for sy in range(h):
        for sx in np.where(mask[sy])[0]:
            if labels[sy, sx] == 0:
                cur += 1
                dq = deque([(sy, sx)])
                labels[sy, sx] = cur
                while dq:
                    y, x = dq.popleft()
                    for dy in (-1, 0, 1):
                        for dx in (-1, 0, 1):
                            ny, nx = y + dy, x + dx
                            if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and labels[ny, nx] == 0:
                                labels[ny, nx] = cur
                                dq.append((ny, nx))
    return labels, cur


def smooth1d(x, k):
    if k <= 1:
        return x.copy()
    ker = np.ones(k) / k
    return np.convolve(np.nan_to_num(x, nan=0.0), ker, mode="same")


def interp_nan(y):
    """NaN 线性插值(用于剖面绘图)."""
    y = np.asarray(y, float)
    idx = np.arange(len(y))
    good = ~np.isnan(y)
    if good.sum() < 2:
        return y
    return np.interp(idx, idx[good], y[good])


# ---------------------------------------------------------------- 分析

def analyze_axis_start(tag, sub, c, d0, rep):
    d, obj, trace = refine_axis(sub, c, d0)
    rep(f"[{tag}] 轴搜索轨迹(每轮最优倾斜/目标值): {trace}")
    rep(f"[{tag}] 收敛轴 d = {d.round(5)}, 截尾RMS = {obj:.4f} mm")
    return d, obj


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "/media/jamesyasr/Shared/大创/点云/test2/1_cld.pcd"
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "out_current")
    os.makedirs(out, exist_ok=True)
    rep_lines = []
    rep = rep_lines.append
    meta = {}

    pts_raw, n_header = read_pcd_xyz(src)
    fin = np.isfinite(pts_raw).all(axis=1)
    pts = pts_raw[fin]
    N = len(pts)
    rep(f"样本: {src}")
    rep(f"PCD 声明点数 = {n_header}, 非有限点 = {int((~fin).sum())}, 有效点 N = {N}")
    lo = np.percentile(pts, 0.05, axis=0)
    hi = np.percentile(pts, 99.95, axis=0)
    rep(f"包围盒(0.05~99.95分位): {lo.round(2)} -> {hi.round(2)}")
    rep(f"包围盒边长 = {(hi - lo).round(1)} mm, 对角线 ≈ {np.linalg.norm(hi - lo):.1f} mm")
    c, vec, ev = pca_axes(pts)
    rep(f"PCA 特征值开方(方向 std, mm) = {np.sqrt(ev).round(2)}")
    for i in range(3):
        rep(f"  主轴{i + 1}: std {np.sqrt(ev[i]):7.2f} mm  方向 {vec[:, i].round(4)}")
    rep("")

    sub = pts[:: max(1, N // 350000)]

    # ---- 双起点轴搜索 -------------------------------------------------
    d1, obj1 = analyze_axis_start("start=PCA1", sub, c, vec[:, 0], rep)
    d2, obj2 = analyze_axis_start("start=PCA2", sub, c, vec[:, 1], rep)
    ang12 = math.degrees(math.acos(abs(float(d1 @ d2))))
    rep(f"两解轴夹角 = {ang12:.2f}°")
    if obj1 <= obj2:
        d_fit, obj_fit, win = d1, obj1, 1
    else:
        d_fit, obj_fit, win = d2, obj2, 2
    rep(f"选择目标值更小者: start=PCA{win} (obj {min(obj1, obj2):.4f} vs "
        f"{max(obj1, obj2):.4f} mm, 差 {(max(obj1, obj2) / min(obj1, obj2) - 1) * 100:.1f}%)")
    rep("")

    # ---- 全量点最终拟合 ------------------------------------------------
    F = fit_cyl_to_points(pts, c, d_fit, iters_gn=12)
    a, b, R = F["a"], F["b"], F["R"]
    u, w, rho, e = F["u"], F["w"], F["rho"], F["e"]
    axis_pt = c + a * F["t1"] + b * F["t2"]
    t_ax = (pts - c) @ d_fit
    phi = np.arctan2(w - b, u - a)
    span, phi_c = circular_span(phi)

    lo_t, hi_t = np.percentile(t_ax, [0.05, 99.95])
    L = hi_t - lo_t
    rep("全量点最终拟合:")
    rep(f"  轴向 d = {d_fit.round(5)}")
    rep(f"  半径 R = {R:.2f} mm")
    rep(f"  轴上参考点 = {axis_pt.round(2)}")
    rep(f"  轴向范围 L = {L:.1f} mm")
    rep(f"  周向覆盖角 span = {math.degrees(span):.2f}°, 弧长 = {R * span:.1f} mm, "
        f"弦长 = {2 * R * math.sin(span / 2):.1f} mm")
    rep(f"  矢高 sagitta = {R * (1 - math.cos(span / 2)):.2f} mm")
    spacing = math.sqrt(R * span * L / N)
    rep(f"  面密度 N/(弧长x轴向) = {N / (R * span * L):.1f} pts/mm^2, "
        f"平均点距 ≈ {spacing:.3f} mm")
    rep(f"  截尾RMS = {F['obj']:.3f} mm")
    med, mad = float(np.median(e)), float(np.median(np.abs(e - np.median(e))))
    sig = 1.4826 * mad
    rep(f"  残差 median={med:.3f} robust sigma={sig:.3f} "
        f"p01={np.percentile(e, 1):.2f} p99={np.percentile(e, 99):.2f} mm")
    meta.update(R=R, span_deg=math.degrees(span), L=L, N=N, sigma=sig, obj=F["obj"])
    rep("")

    # ---- 未选中的另一个解: 其残差结构(碗状系统误差) --------------------
    G = fit_cyl_to_points(sub, c, d2 if win == 1 else d1, iters_gn=10)
    rep(f"落选解(obj较大者)对照: R={G['R']:.1f} mm, obj={G['obj']:.3f} mm")
    rep("")

    # ---- 垂直轴平面内投影占据(弧带形态) --------------------------------
    cw = max((w.max() - w.min()) / 700, spacing * 2)
    cu = max((u.max() - u.min()) / 700, spacing * 2)
    nu2 = int(np.ceil((u.max() - u.min()) / cu)) + 1
    nw2 = int(np.ceil((w.max() - w.min()) / cw)) + 1
    iu2 = np.clip(((u - u.min()) / cu).astype(int), 0, nu2 - 1)
    iw2 = np.clip(((w - w.min()) / cw).astype(int), 0, nw2 - 1)
    occ2 = np.zeros((nu2, nw2), bool)
    occ2[iu2, iw2] = True
    save_gray_mask(occ2.T[::-1], os.path.join(out, "p1_cross_section_arc.png"), scale=2)
    rep("垂直轴平面内投影(弧带图 p1): 横轴=u, 纵轴=w, 1格≈"
        f"{cu:.2f}mm — 展示弧带的矢高弯曲")
    # 弧带矢高实测: 每列 u 的 w 范围中线的弯曲
    col_w_mid = []
    for j in range(nw2):
        sel = occ2[:, j]
        if sel.sum() > 30:
            col_w_mid.append((j, np.where(sel)[0].mean()))
    if len(col_w_mid) > 50:
        arr = np.array(col_w_mid)
        # 二次拟合中线(单位: 格) -> 换算到 mm 后的等效曲率半径
        z = np.polyfit(arr[:, 0], arr[:, 1], 2)
        a_m = cw * z[0] / cu ** 2
        b_m = cw * z[1] / cu
        r_curve = (1 + b_m * b_m) ** 1.5 / abs(2 * a_m)
        rep(f"  弧带中线二次拟合 -> 等效曲率半径 ≈ {r_curve:.0f} mm (与拟合 R={R:.0f} 对照)")
    rep("")

    # ---- 展开面栅格 ------------------------------------------------------
    phi_r = (phi - phi_c + 3 * math.pi) % (2 * math.pi) - math.pi
    ph_lo, ph_hi = np.percentile(phi_r, [0.05, 99.95])
    # 选择栅格: 目标 ~800 格且 cell >= 1mm
    cell_u = max(L / 800, 1.0)
    cell_p = max((ph_hi - ph_lo) / 800, 1.0 / R)
    nu = int(np.ceil(L / cell_u)) + 1
    npp = int(np.ceil((ph_hi - ph_lo) / cell_p)) + 1
    iu = np.clip(((t_ax - lo_t) / cell_u).astype(int), 0, nu - 1)
    ip = np.clip(((phi_r - ph_lo) / cell_p).astype(int), 0, npp - 1)
    occ = np.zeros((nu, npp), bool)
    occ[iu, ip] = True
    sumg = np.zeros((nu, npp))
    np.add.at(sumg, (iu, ip), e)
    cntg = np.zeros((nu, npp), np.int32)
    np.add.at(cntg, (iu, ip), 1)
    with np.errstate(invalid="ignore"):
        egrid = np.where(cntg > 0, sumg / np.maximum(cntg, 1), np.nan)

    vmax_res = max(3 * sig, 0.5)
    em = np.where(occ, np.nan_to_num(egrid, nan=0.0), 0.0)
    save_color_map(em, os.path.join(out, "p2_unwrapped_residual.png"), vmax_res, scale=2)
    save_gray_mask(occ, os.path.join(out, "p3_unwrapped_occupancy.png"), scale=2)
    save_histogram(e, os.path.join(out, "p4_residual_hist.png"),
                   f"radial residual (R={R:.0f}mm, sigma_rob={sig:.2f}mm)", clip=12.0)
    rep(f"展开面栅格 {nu}x{npp} (cell={cell_u:.2f}mm x {math.degrees(cell_p):.3f}°), "
        f"残差图色标 ±{vmax_res:.2f} mm")
    rep("")

    # ---- 多尺度空白分析 -------------------------------------------------
    rep("空白区多尺度分析(展开面):")
    occ1 = occ1_in = None
    row_off1 = col_off1 = 0
    su1 = sp1 = 1.0
    for scale_mm in (0.5, 1.0, 2.0):
        su = max(L / 1200, scale_mm)
        sp = max((ph_hi - ph_lo) / 1200, scale_mm / R)
        mu = int(np.ceil(L / su)) + 1
        mp = int(np.ceil((ph_hi - ph_lo) / sp)) + 1
        mu_i = np.clip(((t_ax - lo_t) / su).astype(int), 0, mu - 1)
        mp_i = np.clip(((phi_r - ph_lo) / sp).astype(int), 0, mp - 1)
        om = np.zeros((mu, mp), bool)
        om[mu_i, mp_i] = True
        rows_any = np.where(om.any(axis=1))[0]
        cols_any = np.where(om.any(axis=0))[0]
        inner = om[rows_any.min():rows_any.max() + 1, cols_any.min():cols_any.max() + 1]
        holes = ~inner
        rep(f"  cell={scale_mm:.1f}mm: 栅格 {mu}x{mp}, 内接区空白率 = {holes.mean() * 100:.1f}%")
        if scale_mm == 1.0:
            occ1, occ1_in = om, inner
            row_off1, col_off1 = int(rows_any.min()), int(cols_any.min())
            su1, sp1 = su, sp
            labels, nl = label_components(holes)
            if nl and holes.sum():
                sizes = np.bincount(labels.ravel())[1:]
                order = np.argsort(sizes)[::-1]
                rep(f"    ≥20格的空白洞: {int((sizes >= 20).sum())} 个")
                rank = 0
                for k in order[:12]:
                    if sizes[k] < 20:
                        break
                    ys, xs = np.where(labels == k + 1)
                    du = (ys.max() - ys.min() + 1) * su
                    dp = (xs.max() - xs.min() + 1) * sp * R
                    cy_ax = lo_t + (row_off1 + ys.mean()) * su
                    aspect = (ys.max() - ys.min() + 1) / max(xs.max() - xs.min() + 1, 1)
                    kind = "圆孔?" if (0.5 <= aspect <= 2.0 and sizes[k] <= 600) else "块状"
                    rep(f"    洞{rank + 1}: {sizes[k]}格 {kind}, ≈{du:.0f}x{dp:.0f}mm, "
                        f"中心轴向≈{cy_ax:.0f}mm")
                    rank += 1
    if occ1 is None:
        occ1, occ1_in = occ, occ
    rep("")

    # ---- 焊缝带: 行/列平均残差剖面 --------------------------------------
    row_cnt = occ.sum(axis=1)
    col_cnt = occ.sum(axis=0)
    masked = np.where(occ, egrid, np.nan)
    row_mean = np.full(nu, np.nan)
    col_mean = np.full(npp, np.nan)
    good_r = row_cnt > 20
    good_c = col_cnt > 20
    if good_r.any():
        row_mean[good_r] = np.nanmean(masked[good_r], axis=1)
    if good_c.any():
        col_mean[good_c] = np.nanmean(masked[:, good_c], axis=0)

    def band_detect(profile, cell, label, min_cells=20):
        valid = ~np.isnan(profile)
        if valid.sum() < 30:
            return []
        v = profile[valid]
        m = np.median(v)
        mad = np.median(np.abs(v - m))
        thr = max(m + 3 * 1.4826 * mad, 0.8)
        flags = np.zeros(len(profile), bool)
        flags[valid] = profile[valid] > thr
        bands = []
        i = 0
        while i < len(flags):
            if flags[i]:
                j = i
                while j + 1 < len(flags) and flags[j + 1]:
                    j += 1
                if (j - i + 1) >= min_cells:
                    bands.append((i, j, float(np.nanmean(profile[i:j + 1]))))
                i = j + 1
            else:
                i += 1
        rep(f"  {label}: thr={thr:.2f}mm, 检出 {len(bands)} 条带")
        for (i0, i1, mg) in bands:
            rep(f"    带: 索引[{i0},{i1}] 位置 {i0 * cell:.0f}..{i1 * cell:.0f} mm, "
                f"宽 {(i1 - i0 + 1) * cell:.0f} mm, 平均残差 {mg:+.2f} mm")
        return bands

    rep("焊缝带检测(展开面平均残差剖面, 稳健阈值):")
    row_bands = band_detect(row_mean, cell_u, "沿周向行(周向走向焊缝应表现于此)", min_cells=10)
    col_bands = band_detect(col_mean, cell_p * R, "沿轴向列(轴向走向条带表现于此)", min_cells=10)
    # 剖面图(NaN 线性插值后平滑)
    prof = smooth1d(interp_nan(row_mean), 3)
    save_line_plot(np.arange(len(prof)) * cell_u, prof,
                   os.path.join(out, "p6_row_mean_profile.png"),
                   "mean radial residual per axial row", "axial position (mm)", "residual (mm)",
                   "(circumferential weld bands show as peaks)")
    prof_c = smooth1d(interp_nan(col_mean), 3)
    save_line_plot(np.arange(len(prof_c)) * cell_p * R, prof_c,
                   os.path.join(out, "p8_col_mean_profile.png"),
                   "mean radial residual per angle column", "angle position (mm arc)",
                   "residual (mm)", "(axial-direction bands show as peaks)")
    rep("")

    # ---- 标定点(圆形空洞)环带凸起 ---------------------------------------
    su, sp = su1, sp1
    lab_h, nl_h = label_components(~occ1_in)
    bump_txt = []
    if nl_h:
        sizes_h = np.bincount(lab_h.ravel())[1:]
        for k in range(nl_h):
            if sizes_h[k] < 20:
                continue
            ys, xs = np.where(lab_h == k + 1)
            hy, hx = ys.max() - ys.min() + 1, xs.max() - xs.min() + 1
            if not (0.5 <= hy / max(hx, 1) <= 2.0 and 5 <= min(hy, hx) <= 30
                    and sizes_h[k] <= 600):
                continue
            mask_h = lab_h == k + 1
            dil = np.zeros_like(mask_h)
            for dy in range(-4, 5):
                for dx in range(-4, 5):
                    if abs(dy) + abs(dx) <= 5:
                        dil |= np.roll(np.roll(mask_h, dy, 0), dx, 1)
            ann = dil & ~mask_h & occ1_in
            if ann.sum() < 30:
                continue
            ay, ax = np.where(ann)
            gy = np.clip(((row_off1 + ay) * su - lo_t) / cell_u, 0, nu - 1).astype(int)
            gx = np.clip(((col_off1 + ax) * sp - ph_lo) / cell_p, 0, npp - 1).astype(int)
            vals = egrid[gy, gx]
            vals = vals[np.isfinite(vals)]
            if len(vals) < 30:
                continue
            cy_ax = lo_t + (row_off1 + ys.mean()) * su
            dia_mm = max(hy, hx) * su
            bump_txt.append((dia_mm, cy_ax, float(np.mean(vals)), len(vals)))
    if bump_txt:
        bump_txt.sort(reverse=True)
        rep("圆形空洞(标定点)及其环带径向残差(凸起证据):")
        for dia, cy, bp, nv in bump_txt[:16]:
            rep(f"  洞@轴向≈{cy:.0f}mm 直径≈{dia:.0f}mm: 环带平均残差 {bp:+.2f} mm "
                f"(全局中位数 {med:+.2f}, N={nv})")
    else:
        rep("圆形空洞(标定点): 未检出明显圆孔环带结构")
    rep("")

    # ---- 边界直线性 ------------------------------------------------------
    row_l, row_r = np.full(nu, np.nan), np.full(nu, np.nan)
    for i in range(nu):
        cs = np.where(occ[i])[0]
        if len(cs) > 50:
            row_l[i], row_r[i] = cs[0], cs[-1]
    def edge_resid(arr, unit_mm):
        idx = np.where(~np.isnan(arr))[0]
        if len(idx) < 50:
            return float("nan")
        v = arr[idx]
        z = np.polyfit(idx, v, 1)
        return float(np.std(v - np.polyval(z, idx)) * unit_mm)
    rep("边界直线性(去趋势后的残差 std):")
    for name, arr, unit in (("周向左边界", row_l, cell_u), ("周向右边界", row_r, cell_u)):
        rep(f"  {name}: std = {edge_resid(arr, unit):.2f} mm")
    col_t, col_b = np.full(npp, np.nan), np.full(npp, np.nan)
    for j in range(npp):
        rs = np.where(occ[:, j])[0]
        if len(rs) > 50:
            col_t[j], col_b[j] = rs[0], rs[-1]
    arc_per_cell = R * cell_p  # 每格弧长 mm
    for name, arr in (("轴向左边界", col_t), ("轴向右边界", col_b)):
        rep(f"  {name}: std = {edge_resid(arr, arc_per_cell):.2f} mm (弧长)")
    rep("")

    # ---- 小弧段退化: 最终轴附近的倾斜景观 + 半径耦合 -------------------
    L3 = np.zeros((13, 13))
    degs = np.linspace(-3, 3, 13)
    rads = []
    for ia, al_d in enumerate(degs):
        for ib, be_d in enumerate(degs):
            dd = axis_with_tilt(d_fit, math.radians(al_d), math.radians(be_d))
            ff = fit_cyl_to_points(sub, c, dd, iters_gn=6)
            L3[ib, ia] = ff["obj"]
            if abs(al_d) < 1e-9 and abs(be_d) < 1e-9:
                r_center = ff["R"]
            if ib == 6:
                rads.append(ff["R"])
    save_heatmap(L3, os.path.join(out, "p5_tilt_landscape.png"),
                 f"tilt landscape around final axis (trimmed RMS, start=PCA{win})",
                 "alpha (deg)", "beta (deg)")
    save_line_plot(degs, rads, os.path.join(out, "p7_radius_vs_tilt.png"),
                   "fitted radius vs axis tilt", "tilt alpha (deg)", "R (mm)",
                   "(beta=0; radius-direction coupling)")
    rel = (L3.max() - L3.min()) / max(L3.min(), 1e-12)
    rep("小弧段退化量化(最终轴附近 ±3° 景观):")
    rep(f"  obj: min={L3.min():.3f} max={L3.max():.3f} mm, 相对波动 {rel * 100:.1f}%")
    rep(f"  拟合半径随倾斜 alpha(-3°..+3°) 变化: {min(rads):.0f} .. {max(rads):.0f} mm "
        f"(波动 {max(rads) - min(rads):.0f} mm, 中心 R={r_center:.0f} mm)")
    rep("")

    with open(os.path.join(out, "report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(rep_lines))
    with open(os.path.join(out, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    print("\n".join(rep_lines))
    print(f"\n[OK] 输出目录: {out}")


if __name__ == "__main__":
    main()
