#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成"两朵云"真实感焊前装配样本(远件/近件各自一份点云), 供软件与验证程序使用。

依据: 真实远件扫描 1_pb.pcd 的实测特征(半径按图纸 1940)
  设计半径 R=1940mm(图纸值), 点距 0.41mm(6.0 点/mm²), 点噪声 σ=0.063mm,
  形面谱 2mm:0.030 / 10mm:0.045 / 50mm:0.161 / 100mm:0.282 mm,
  足迹 ~240x250mm 近矩形、四边渐稀(最外 3mm 占用 3~8%)、内部无空洞、无尖刺。
生成方式: 在展开面 (a,s) 上按栅格撒点 -> 叠加形面与噪声 -> 按真实传感器
  (距离/入射角/遮挡)做可见性筛选 -> 圆柱回投 -> 写二进制 PCD(float32)。

几何约定(任意缝角 psi, 90°=环缝):
  缝方向 t=(cos psi, sin psi), 跨缝法向 n=(sin psi, cos psi)  (在展开面 (a,s) 内)
  跨缝坐标 x = a*sin(psi) + s*cos(psi), 沿缝坐标 y = a*cos(psi) - s*sin(psi)
  远件: 材料在 x>=0  (接头边 x=0)          e = form
  近件: 材料在 x<=-g (接头边 x=-g)         e = step + form   (step>0 = 近件在外)
  真值: 径向阶差 = step(近件在外为正); 沿跨缝方向的间隙 = g
        psi=90° 时跨缝方向=轴向 -> 软件用"沿圆柱轴向"模式;
        psi 非 90° 时跨缝方向不等于轴向 -> 软件用"曲面内接缝法向"模式
"""
import os, sys, math, argparse
import numpy as np

def cyl_point(a, s, e, R, u, v, w, O):
    ph = s / R
    return (O[None, :] + a[:, None] * u[None, :]
            + (R + e)[:, None] * (np.cos(ph)[:, None] * v[None, :] + np.sin(ph)[:, None] * w[None, :]))

def form_field(a, s, scales):
    f = np.zeros_like(a)
    for amp, lam_a, lam_s, ph in scales:
        f += amp * np.sin(2 * math.pi * a / lam_a + ph) * np.cos(2 * math.pi * s / lam_s + ph * 0.7)
    return f

def raster(a0, a1, s0, s1, pitch, jitter, rng):
    ns = int(round((s1 - s0) / pitch))
    na = int(round((a1 - a0) / pitch))
    aa = a0 + (np.arange(na) + 0.5) * pitch
    ss = s0 + (np.arange(ns) + 0.5) * pitch
    A, S = np.meshgrid(aa, ss, indexing='ij')
    A = A.ravel(); S = S.ravel()
    odd = (np.arange(na)[:, None] % 2 == 1).repeat(ns, 1).ravel()
    S = S + odd * pitch * 0.5                      # 隔行错开(真实光栅)
    A = A + rng.normal(0, jitter, A.size)          # 走位抖动
    S = S + rng.normal(0, jitter, S.size)
    return A, S

def write_pcd(path, pts):
    n = len(pts)
    hdr = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
           "TYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
           "POINTS %d\nDATA binary\n" % (n, n))
    with open(path, 'wb') as f:
        f.write(hdr.encode('ascii'))
        f.write(np.ascontiguousarray(pts, dtype='<f4').tobytes())

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'samples_realistic'))
    ap.add_argument('--only', default='')
    A = ap.parse_args()
    R_design = 1940.0        # 设计半径(图纸值)
    pitch, jitter, sigma = 0.41, 0.03, 0.063
    # 形面谱(幅值 mm, 轴向波长, 周向波长, 相位)
    scales = [(0.25, 200.0, 260.0, 0.3), (0.15, 120.0, 90.0, 1.1), (0.08, 45.0, 60.0, 2.0), (0.03, 15.0, 18.0, 0.7)]
    # 传感器: 距表面 H mm, 矩形视场(跨缝 ±17°, 沿缝 ±13°), 视场边缘 6% 渐稀;
    #         再加最大入射角/量程限制 -> 足迹约 230x255 mm, 与实测 237x248 mm 相当
    H, fov_a, fov_s = 550.0, 17.0, 13.0
    inc_max, range_max = math.radians(62.0), 2000.0
    axis_off_edge = 60.0      # 视场轴心离接头边的距离(保证接头边在视场内)
    occlude = False           # 两件分开扫描 -> 默认无互相遮蔽
    cases = [
        # 只保留一对(用户要求). 环缝, 间隙 5mm, 径向阶差 +3mm(近件明显在外), 更接近真实现场量级
        ('ring_g5_de+3',            90.0,  5.0,  3.0, 120.0, 0.0),
    ]
    os.makedirs(A.out, exist_ok=True)
    for name, psi, g, step, near_w, offs in cases:
        if A.only and A.only != name:
            continue
        rng = np.random.default_rng(hash(name) % (2**32))
        # 筒体轴线取任意方向(真实零件轴线不会与坐标轴平行); 近远由用户选择, 与 Z 无关
        u = np.array([0.37, 0.48, 0.79]); u /= np.linalg.norm(u)
        v = np.cross(u, np.array([0.0, 0.0, 1.0])); v /= np.linalg.norm(v)
        w = np.cross(u, v); w /= np.linalg.norm(w)
        O = np.zeros(3)
        psi_r = math.radians(psi)
        # 环缝(psi=90): 接头边沿 s 方向 -> 材料沿 a 分开; 纵缝(psi=0): 材料沿 s 分开
        # 本文件统一按"跨缝方向"= 展开面内与缝垂直的方向构造: 环缝时跨缝=轴向(a)
        far_span_cross, far_span_along = 250.0, 240.0     # 远件: 跨缝 250 x 沿缝 240
        # 在"跨缝坐标 (x,y)"下撒点: 材料边界 x=0 / x=-g 正好落在格线上(消除半格偏差)
        n_a = np.array([math.sin(psi_r), math.cos(psi_r)])   # 跨缝法向(a,s)
        t_a = np.array([math.cos(psi_r), -math.sin(psi_r)])  # 沿缝方向(a,s)
        def xy_to_as(X, Y):      # (n,t) 正交基的逆变换
            return X * n_a[0] + Y * t_a[0], X * n_a[1] + Y * t_a[1]
        def cross(Aa, Ss):
            return Aa * n_a[0] + Ss * n_a[1]
        def along(Aa, Ss):
            return Aa * t_a[0] + Ss * t_a[1]
        half_along = 140.0
        Xf, Yf = raster(0.0, far_span_cross, -half_along, half_along, pitch, jitter, rng)
        Xn, Yn = raster(-g - near_w, -g, -half_along, half_along, pitch, jitter, rng)
        Fa, Fs = xy_to_as(Xf, Yf)
        Na, Ns = xy_to_as(Xn, Yn)
        out = {}
        for tag, (Aa, Ss, e0) in (('far', (Fa, Fs, 0.0)), ('near', (Na, Ns, step))):
            xa = cross(Aa, Ss); ya = along(Aa, Ss)
            e = e0 + form_field(Aa, Ss, scales) + rng.normal(0, sigma, Aa.size)
            pts = cyl_point(Aa, Ss, e, R_design, u, v, w, O)
            x_c = (axis_off_edge if tag == 'far' else -g - axis_off_edge) + offs
            a_c, s_c = float(xy_to_as(np.array([x_c]), np.array([0.0]))[0][0]), \
                       float(xy_to_as(np.array([x_c]), np.array([0.0]))[1][0])
            Sc = cyl_point(np.array([a_c]), np.array([s_c]), np.array([H]), R_design, u, v, w, O)[0]
            dvec = pts - Sc
            dist = np.linalg.norm(dvec, axis=1)
            ph = Ss / R_design
            nrm = np.cos(ph)[:, None] * v + np.sin(ph)[:, None] * w
            cosi = np.abs(np.einsum('ij,ij->i', np.where(dist[:, None] > 0, dvec, 1e-9), nrm)) / np.maximum(dist, 1e-9)
            ang_x = np.arctan2(np.abs(xa - x_c), H + np.abs(e))
            ang_y = np.arctan2(np.abs(ya), H + R_design)
            hard = (dist <= range_max) & (ang_x <= math.radians(fov_a)) & (ang_y <= math.radians(fov_s))
            hard &= (cosi >= math.cos(inc_max))
            r = np.maximum(ang_x / math.radians(fov_a), ang_y / math.radians(fov_s))
            soft = np.clip((1.0 - r) / 0.06, 0.0, 1.0)
            vis = hard & (rng.random(Aa.size) < soft)
            # 遮蔽(仅"装配后整体扫描"场景, 且传感器在近件一侧时才存在):
            #   阴影长度 = 阶差 × tan(入射角); 垂直观察无阴影
            if occlude and tag == 'far' and step > 0 and x_c < 0.0:
                beta = np.arccos(np.clip(cosi, -1, 1))
                shadow = step * np.tan(beta)
                vis &= ~((xa > 0) & (xa < np.minimum(shadow, 25.0)))
            # 接头边是材料边: 密集到边(最后一小段略稀)
            vis &= rng.random(Aa.size) < (1.0 - 0.12 * np.exp(-np.abs(xa - (0.0 if tag == 'far' else -g)) / 0.5))
            vis &= rng.random(Aa.size) > 0.003
            sel = np.nonzero(vis)[0]
            P3 = pts[sel]
            out[tag] = (P3, xa[sel], ya[sel])
            d = os.path.join(A.out, name); os.makedirs(d, exist_ok=True)
            write_pcd(os.path.join(d, tag + '.pcd'), P3)
        fp, fa, fs_ = out['far']; np_, na, ns_ = out['near']
        with open(os.path.join(A.out, name, 'truth.txt'), 'w') as f:
            f.write("case %s\n" % name)
            f.write("seam_psi_deg %.1f   # 90=环缝(缝沿周向, 跨缝=轴向); 0=纵缝\n" % psi)
            f.write("R_design_mm %.4f\nR_true_mm %.4f\n" % (R_design, R_design))
            f.write("gap_axial_mm %.4f        # 间隙(沿轴向)真值\n" % g)
            f.write("step_radial_mm %.4f      # 径向阶差真值(近件在外为正)\n" % step)
            f.write("noise_sigma_mm %.4f\npitch_mm %.4f\n" % (sigma, pitch))
            f.write("far_points %d\nnear_points %d\n" % (len(fp), len(np_)))
            f.write("far_span_cross_mm %.1f\nfar_span_along_mm %.1f\n" % (np.ptp(fa), np.ptp(fs_)))
            f.write("near_span_cross_mm %.1f\nnear_span_along_mm %.1f\n" % (np.ptp(na), np.ptp(ns_)))
            f.write("measure_direction_mode %s\n" % ("沿圆柱轴向" if abs(psi - 90.0) < 1e-6 else "曲面内接缝法向"))
            view_cross_lo = axis_off_edge + offs - H * math.tan(math.radians(fov_a))
            f.write("far_view_cross_lo_mm %.1f\n" % view_cross_lo)
            f.write("expect %s\n" % ("measurable" if view_cross_lo <= 0.0 else "unmeasurable_edge_out_of_view"))
            f.write("sensor_H_mm %.1f  inc_max_deg %.1f  range_max_mm %.1f  lateral_offset_mm %.1f\n"
                    % (H, math.degrees(inc_max), range_max, offs))
            f.write("use: 远件=%s  近件=%s  跨缝方向=%s(%s)\n"
                    % ('far.pcd', 'near.pcd',
                       "轴向" if abs(psi - 90.0) < 1e-6 else "接缝法向",
                       "缝走向 %.0f°" % psi))
        print("[gen] %-24s far=%7d near=%7d  (g=%.1f, step=%+.1f)" % (name, len(fp), len(np_), g, step))

main()
