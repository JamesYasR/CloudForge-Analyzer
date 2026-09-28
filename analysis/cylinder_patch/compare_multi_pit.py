#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compare_multi_pit.py -- 把 test_cyl_pothole 的 [PIT] 输出与真值 JSON 比对

用法:
  python3 compare_multi_pit.py <run_log> <truth_json>
  python3 compare_multi_pit.py <run_log> --truth-dir   # 自动找同目录 truth.json

展开域坐标对齐:
  生成脚本用 AXIS_DIR, 拟合得到的轴可能与其反向(±180° 等价). 反向时
  轴向翻转 + 周向翻转 -> (a_cpp, s_cpp) = (-a_gen, -s_gen).
  脚本对两种取向都试, 取总中心误差更小者.
"""
import json
import math
import re
import sys


def parse_pits(path):
    pits, meta = [], {}
    for line in open(path, encoding="utf-8", errors="replace"):
        if line.startswith("[PITCOUNT] "):
            m = dict(re.findall(r"(\w+)=([-\d.eE+]+)", line))
            meta = {k: float(v) for k, v in m.items()}
        elif line.startswith("[PIT] "):
            body = line.rstrip("\n")
            reason = re.search(r"reason=(.*)$", body)
            body = re.sub(r"reason=.*$", "", body)   # reason 是自由文本, 先剥离
            kv = dict(re.findall(r"(\w+)=([-\d.eE+]+)", body))
            d = {k: float(v) for k, v in kv.items()}
            d["reason"] = reason.group(1) if reason else "-"
            pits.append(d)
    return pits, meta


def map_truth(a, s, flip):
    return (-a, -s) if flip else (a, s)


def main():
    log = sys.argv[1]
    truth_path = sys.argv[2]
    pits, meta = parse_pits(log)
    truth = json.load(open(truth_path, encoding="utf-8"))
    dents = truth["dents"]
    if not pits:
        print("未检出任何 [PIT] 行 -> 检出 0 个凹坑")
        return 0

    def cost(flip):
        tot = 0.0
        for d in dents:
            ta, ts = map_truth(d["center_a"], d["center_s"], flip)
            best = min(math.hypot(ta - p["ec_a"], ts - p["ec_s"]) for p in pits)
            tot += best
        return tot

    flip = cost(True) < cost(False)
    print(f"展开域对齐: {'(a,s) -> (-a,-s)' if flip else '(a,s) 原样'} "
          f"(拟合轴与生成轴{'反向' if flip else '同向'})")
    print(f"检出坑数 = {len(pits)} (cluster_count={meta.get('cluster_count', '?')}, "
          f"cell={meta.get('cell', '?')}mm, W={meta.get('win', '?')}mm)\n")

    rows = []
    used = set()
    for d in dents:
        ta, ts = map_truth(d["center_a"], d["center_s"], flip)
        # 按中心最近匹配(且一个实测坑只匹配一个真值坑)
        cand = sorted((p for p in pits if p["idx"] not in used),
                      key=lambda p: math.hypot(ta - p["ec_a"], ts - p["ec_s"]))
        if not cand:
            rows.append((d, None, None))
            continue
        p = cand[0]
        used.add(p["idx"])
        rows.append((d, p, math.hypot(ta - p["ec_a"], ts - p["ec_s"])))

    hdr = (f"{'真值坑':<12}{'真值深':>8}{'实测深':>9}{'误差':>9}"
           f"{'真值长轴':>10}{'实测':>9}{'误差%':>9}"
           f"{'真值短轴':>10}{'实测':>9}{'误差%':>9}{'中心误差':>10}{'点数':>8}{'判定':>6}")
    print(hdr)
    print("-" * len(hdr))
    worst = 0.0
    for d, p, cerr in rows:
        tl_a, tl_b = 2 * d["semi_a"], 2 * d["semi_b"]  # 生成脚本: semi_a 对应周向(s)
        if p is None:
            print(f"{d['name']:<12}{d['depth']:>8.3f}{'--':>9}{'--':>9}{tl_a:>10.1f}{'--':>9}"
                  f"{'--':>9}{tl_b:>10.1f}{'--':>9}{'--':>9}{'--':>10}{'--':>8}{'漏检':>6}")
            continue
        dep_err = p["local_max"] - d["depth"]
        # 椭圆长短轴与真值长短轴配对(实测长短轴按数值排序, 真值按 semi_a/semi_b 排序)
        meas = sorted([p["major"], p["minor"]], reverse=True)
        tru = sorted([tl_a, tl_b], reverse=True)
        la_e = (meas[0] - tru[0]) / tru[0] * 100.0
        lb_e = (meas[1] - tru[1]) / tru[1] * 100.0
        print(f"{d['name']:<12}{d['depth']:>8.3f}{p['local_max']:>9.3f}{dep_err:>+9.3f}"
              f"{tru[0]:>10.1f}{meas[0]:>9.1f}{la_e:>+9.1f}"
              f"{tru[1]:>10.1f}{meas[1]:>9.1f}{lb_e:>+9.1f}"
              f"{cerr:>10.1f}{int(p['n']):>8}{('OK' if p['valid'] else '拒'):>6}")

    # 多余检出(未匹配到真值的实测坑)
    extra = [p for p in pits if p["idx"] not in used]
    if extra:
        print("\n未匹配到真值的多余检出(误报):")
        for p in extra:
            print(f"  idx={int(p['idx'])} n={int(p['n'])} local_max={p['local_max']:.3f} "
                  f"major={p['major']:.1f} minor={p['minor']:.1f} "
                  f"中心(a,s)=({p['ec_a']:.1f},{p['ec_s']:.1f}) valid={int(p['valid'])}")
    else:
        print("\n无多余检出(无误报)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
