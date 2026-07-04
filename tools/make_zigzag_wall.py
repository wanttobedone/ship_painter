#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_zigzag_wall.py  —  由环绕式分层航点(waypoints_full.csv)离线后处理出
                        "只喷正对无人机那面墙"的竖向 zigzag(弓字形)路径。

定位
----
纯离线后处理，python3.8，依赖 numpy + 标准库，**不 import ros**。
仅用于"飞手看 + QGC 真机"链路，不涉及机载自主飞行，不改任何 C++/launch。

思路
----
waypoints_full.csv 含 31 层 × 0.01m 环形航点及 mission0 系法向。正对无人机的墙 =
法向指向无人机的点(dot(n, -x̂) > 阈值)。每层近墙点是一条横向线段 → 在其上按列位置 y_k
线性插值取点 → 每列串起各层成一根竖线 → 列间首尾交替相连 = 竖向 zigzag。
输出沿用现有 CSV 表头(layer_index,wp_index,x,y,z,yaw_deg,nx,ny,nz)，其中 layer_index
语义变为"列号"，从而 make_pilot_viewer.py / waypoints_to_qgc_plan.py 可直接复用。

坐标系与环绕版完全一致(mission0 相对系: x=前指向建筑, y=左, z=离地高)。摆机规则相同。

用法
----
  python3.8 tools/make_zigzag_wall.py \
      --csv /tmp/ship_painter_export/waypoints_full.csv \
      --meta /tmp/ship_painter_export/export_meta.json \
      [--pattern vertical|horizontal] [--column-spacing 2.0] [--edge-inset 1.0] \
      [--normal-dot 0.7] [--pilot-corner-angle-deg 10] [--pilot-max-gap 10] [--out-dir DIR]

  自测:  python3.8 tools/make_zigzag_wall.py --self-test
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

NEAR_WALL_DIR = np.array([-1.0, 0.0, 0.0])  # 正对无人机的墙: 法向指向 -x(无人机在建筑前方)


# ---------------------------------------------------------------------------
def yaw_from_normal(nx, ny):
    """与 C++ applyMissionTransform 一致: heading=-n, yaw=atan2(-ny,-nx) (度)。"""
    return math.degrees(math.atan2(-ny, -nx))


def read_full_csv(path):
    """读 waypoints_full.csv, 按 layer_index 分组(保持 wp_index 顺序)。
    返回 OrderedDict{layer_index: np.array(M,7)}  列: x,y,z,yaw,nx,ny,nz"""
    from collections import OrderedDict
    buckets = OrderedDict()
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        need = {"layer_index", "wp_index", "x", "y", "z", "yaw_deg", "nx", "ny", "nz"}
        if not need.issubset(set(reader.fieldnames or [])):
            raise ValueError("CSV 缺列, 需要 %s" % sorted(need))
        for r in reader:
            li = int(float(r["layer_index"]))
            buckets.setdefault(li, []).append([
                float(r["x"]), float(r["y"]), float(r["z"]), float(r["yaw_deg"]),
                float(r["nx"]), float(r["ny"]), float(r["nz"])])
    return OrderedDict((k, np.asarray(v, dtype=np.float64)) for k, v in buckets.items())


def longest_circular_run(mask):
    """环形布尔序列里最长连续 True 段的索引列表(首尾相邻视为连续)。"""
    n = len(mask)
    if n == 0:
        return []
    if mask.all():
        return list(range(n))
    # 从某个 False 之后打断环
    false_idx = np.where(~mask)[0]
    start = (false_idx[0] + 1) % n
    order = [(start + i) % n for i in range(n)]
    best, cur = [], []
    for idx in order:
        if mask[idx]:
            cur.append(idx)
        else:
            if len(cur) > len(best):
                best = cur
            cur = []
    if len(cur) > len(best):
        best = cur
    return best


def near_wall_segment(layer_pts, normal_dot, expected_x, near_x_tol):
    """取一层的近墙段(最长连续段)。返回按 y 升序排列的 (x,y,z,yaw,nx,ny,nz) 数组或 None。
    筛选 = 法向指向无人机(dot(n,-x̂)>normal_dot) 且 x 落在主墙面附近
    (|x-expected_x|<=near_x_tol, 用于剔除建筑顶部退台/凹进等"另一面墙", 避免竖列大跳变)。"""
    dotv = -(layer_pts[:, 4])  # dot(n, (-1,0,0)) = -nx
    mask = (dotv > normal_dot) & (np.abs(layer_pts[:, 0] - expected_x) <= near_x_tol)
    if not mask.any():
        return None
    idxs = longest_circular_run(mask)
    if len(idxs) < 2:
        return None
    seg = layer_pts[idxs]                       # (m,7): x,y,z,yaw,nx,ny,nz
    ys = seg[:, 1]
    order = np.argsort(ys)                       # 按 y 升序, 供 np.interp
    seg = seg[order]
    return seg  # 已按 y 升序


def interp_at_y(seg, y_k):
    """在近墙段上按 y 线性插值一点, y_k 超出范围返回 None(不外推)。
    返回 [x,y,z,yaw,nx,ny,nz]。"""
    ys = seg[:, 1]
    if y_k < ys[0] - 1e-9 or y_k > ys[-1] + 1e-9:
        return None
    x = np.interp(y_k, ys, seg[:, 0])
    z = np.interp(y_k, ys, seg[:, 2])
    nx = np.interp(y_k, ys, seg[:, 4])
    ny = np.interp(y_k, ys, seg[:, 5])
    nz = np.interp(y_k, ys, seg[:, 6])
    nrm = math.sqrt(nx * nx + ny * ny + nz * nz)
    if nrm > 1e-9:
        nx, ny, nz = nx / nrm, ny / nrm, nz / nrm
    return [x, y_k, z, yaw_from_normal(nx, ny), nx, ny, nz]


# ---------------------------------------------------------------------------
def build_vertical(layers_sorted, column_spacing, edge_inset, normal_dot, expected_x, near_x_tol, warn):
    """竖向: 返回 columns = [np.array(M,7), ...] 按列, 弓字交替。"""
    segs = []
    for z, pts in layers_sorted:
        seg = near_wall_segment(pts, normal_dot, expected_x, near_x_tol)
        if seg is None:
            segs.append(None)
            continue
        mx = float(np.mean(seg[:, 0]))
        if abs(mx - expected_x) > 1.0:
            warn("层 z=%.2f 近墙段 x 均值=%.2f 偏离预期 %.2f >1m" % (z, mx, expected_x))
        segs.append(seg)
    n_skipped = sum(1 for s in segs if s is None)
    if n_skipped:
        warn("%d 层无主墙面近墙点被跳过(顶部退台/凹进, 或 normal-dot/near-x-tol 过严)" % n_skipped)
    valid = [s for s in segs if s is not None]
    if not valid:
        raise RuntimeError("没有任何层筛出近墙段, 请调小 --normal-dot 或检查 CSV")
    y_left = max(float(s[:, 1].max()) for s in valid)   # 左 = y 最大
    y_right = min(float(s[:, 1].min()) for s in valid)  # 右 = y 最小

    # 列位置: 从左上向右, y_k = y_left - inset - k*spacing
    y_positions = []
    y_k = y_left - edge_inset
    while y_k >= y_right + edge_inset - 1e-9:
        y_positions.append(y_k)
        y_k -= column_spacing

    columns = []
    for k, yk in enumerate(y_positions):
        col = []
        for seg in segs:                         # segs 已按层 高->低
            if seg is None:
                continue
            p = interp_at_y(seg, yk)
            if p is not None:
                col.append(p)
        if len(col) < 2:
            continue
        if k % 2 == 1:                           # 奇数列反向 => 弓字(偶列向下, 奇列向上)
            col = col[::-1]
        columns.append(np.asarray(col, dtype=np.float64))
    return columns


def build_horizontal(layers_sorted, normal_dot, expected_x, near_x_tol, warn):
    """横排: 每层近墙段作为一行, 行内方向逐层交替(最高层 左->右)。返回 (rows, layer_ids)。"""
    rows, layer_ids = [], []
    for j, (z, pts) in enumerate(layers_sorted):
        seg = near_wall_segment(pts, normal_dot, expected_x, near_x_tol)  # 已按 y 升序
        if seg is None:
            continue
        mx = float(np.mean(seg[:, 0]))
        if abs(mx - expected_x) > 1.0:
            warn("层 z=%.2f 近墙段 x 均值=%.2f 偏离预期 %.2f >1m" % (z, mx, expected_x))
        # 左=y大. 最高层(j=0) 左->右 = y 降序; 逐层交替
        row = seg[::-1] if (j % 2 == 0) else seg.copy()
        # 重算 yaw
        out = []
        for r in row:
            out.append([r[0], r[1], r[2], yaw_from_normal(r[4], r[5]), r[4], r[5], r[6]])
        rows.append(np.asarray(out, dtype=np.float64))
        layer_ids.append(j)
    return rows, layer_ids


# ---------------------------------------------------------------------------
def downsample(points, corner_deg, max_gap):
    """每列/行降采样: 首末必留 + 拐点(方向变化>阈值) + 间隔>max_gap 补点。"""
    n = len(points)
    if n <= 2:
        return points.copy()
    pos = points[:, :3]
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[-1] = True
    cr = math.radians(corner_deg)
    for i in range(1, n - 1):
        vin = pos[i] - pos[i - 1]
        vout = pos[i + 1] - pos[i]
        ln, lo = np.linalg.norm(vin), np.linalg.norm(vout)
        if ln < 1e-9 or lo < 1e-9:
            continue
        cosang = np.clip(vin.dot(vout) / (ln * lo), -1.0, 1.0)
        if math.acos(cosang) > cr:
            keep[i] = True
    anchors = np.where(keep)[0]
    out = [points[anchors[0]].copy()]
    for m in range(len(anchors) - 1):
        ka, kb = anchors[m], anchors[m + 1]
        seg = pos[ka:kb + 1]
        seglens = np.linalg.norm(np.diff(seg, axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seglens)])
        L = cum[-1]
        if L > max_gap:
            nmarks = int(L // max_gap)
            for t in range(1, nmarks + 1):
                target = t * max_gap
                if target >= L - 1e-9:
                    break
                j = max(0, min(int(np.searchsorted(cum, target) - 1), len(seg) - 2))
                lt = (target - cum[j]) / max(cum[j + 1] - cum[j], 1e-9)
                p = points[ka + j] + lt * (points[ka + j + 1] - points[ka + j])
                nrm = math.sqrt(p[4] ** 2 + p[5] ** 2 + p[6] ** 2)
                if nrm > 1e-9:
                    p[4:7] /= nrm
                p[3] = yaw_from_normal(p[4], p[5])
                out.append(p)
        out.append(points[kb].copy())
    return np.asarray(out)


def write_csv(path, groups, layer_ids=None):
    with open(path, "w", newline="") as f:
        f.write("layer_index,wp_index,x,y,z,yaw_deg,nx,ny,nz\n")
        for gi, g in enumerate(groups):
            lid = gi if layer_ids is None else layer_ids[gi]
            for wi, p in enumerate(g):
                f.write("%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n"
                        % (lid, wi, p[0], p[1], p[2], p[3], p[4], p[5], p[6]))


def total_distance(groups):
    d = 0.0
    prev = None
    for g in groups:
        for p in g:
            if prev is not None:
                d += float(np.linalg.norm(p[:3] - prev))
            prev = p[:3]
    return d


# ---------------------------------------------------------------------------
def run(args, meta, layers, warn):
    layer_height = float(meta["layer_height"])
    fwd = float(meta["mission_forward_distance"])
    spray = float(meta["spray_distance"])
    expected_x = fwd - spray

    column_spacing = args.column_spacing if args.column_spacing is not None else layer_height
    edge_inset = args.edge_inset if args.edge_inset is not None else column_spacing / 2.0

    # 层按 z 均值 高->低 显式排序(不依赖源文件顺序)
    layers_sorted = sorted(
        ((float(np.mean(pts[:, 2])), pts) for pts in layers.values()),
        key=lambda t: -t[0])

    if args.pattern == "vertical":
        groups = build_vertical(layers_sorted, column_spacing, edge_inset,
                                 args.normal_dot, expected_x, args.near_x_tol, warn)
        layer_ids = None
        num_columns = len(groups)
    else:
        groups, layer_ids = build_horizontal(layers_sorted, args.normal_dot,
                                              expected_x, args.near_x_tol, warn)
        num_columns = len(groups)

    pilot = [downsample(g, args.pilot_corner_angle_deg, args.pilot_max_gap) for g in groups]
    return groups, pilot, layer_ids, num_columns, column_spacing, edge_inset, expected_x


def main():
    ap = argparse.ArgumentParser(
        description="环绕分层航点 -> 单墙竖向 zigzag 路径(离线后处理)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--csv", help="waypoints_full.csv")
    ap.add_argument("--meta", help="export_meta.json")
    ap.add_argument("--pattern", choices=["vertical", "horizontal"], default="vertical")
    ap.add_argument("--column-spacing", type=float, default=None, help="列间距 m (默认=layer_height)")
    ap.add_argument("--edge-inset", type=float, default=None, help="列距墙左右边缘内缩 m (默认=列距/2)")
    ap.add_argument("--normal-dot", type=float, default=0.7, help="近墙筛选阈值 dot(n,-x̂)>该值")
    ap.add_argument("--near-x-tol", type=float, default=3.0,
                    help="主墙面 x 容差 m: 仅保留 |x-(forward-spray)|<=该值 的点, 剔除顶部退台/凹进; 设大(如100)可关闭")
    ap.add_argument("--pilot-corner-angle-deg", type=float, default=10.0)
    ap.add_argument("--pilot-max-gap", type=float, default=10.0, help="飞手版列内最大点距 m")
    ap.add_argument("--out-dir", default=None, help="输出目录(默认=CSV 所在目录)")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        sys.exit(self_test())

    if not args.csv or not args.meta:
        ap.error("需要 --csv 与 --meta (或 --self-test)")

    with open(args.meta) as f:
        meta = json.load(f)
    layers = read_full_csv(args.csv)

    warnings = []
    def warn(m):
        warnings.append(m)
        print("WARNING: " + m)

    groups, pilot, layer_ids, num_cols, col_sp, inset, expx = run(args, meta, layers, warn)

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.csv))
    os.makedirs(out_dir, exist_ok=True)
    full_path = os.path.join(out_dir, "zigzag_wall_full.csv")
    pilot_path = os.path.join(out_dir, "zigzag_wall_pilot.csv")
    meta_path = os.path.join(out_dir, "zigzag_meta.json")
    write_csv(full_path, groups, layer_ids)
    write_csv(pilot_path, pilot, layer_ids)

    zmeta = dict(meta)
    zmeta.update({
        "path_style": "zigzag",
        "pattern": args.pattern,
        "column_spacing": col_sp,
        "edge_inset": inset,
        "num_columns": num_cols,
        "wall": "near(+x facing drone)",
        "start": "top-left, first stroke down",
    })
    with open(meta_path, "w") as f:
        json.dump(zmeta, f, ensure_ascii=False, indent=2)

    counts = [len(g) for g in groups]
    pcounts = [len(g) for g in pilot]
    tot = sum(counts)
    ptot = sum(pcounts)
    dist = total_distance(groups)
    first = groups[0][0] if groups else None
    last = groups[-1][-1] if groups else None
    print("已写出:\n  %s\n  %s\n  %s" % (full_path, pilot_path, meta_path))
    print("--------------------------------------------------")
    print("模式               : %s" % args.pattern)
    print("列/行数            : %d" % num_cols)
    print("每列点数(全量)     : %d ~ %d" % (min(counts), max(counts)))
    print("总点数(全量/飞手)  : %d / %d" % (tot, ptot))
    print("总里程             : %.1f m" % dist)
    print("近墙段 x 预期       : %.3f m (= forward - spray)" % expx)
    if first is not None:
        print("首点 rel           : x=%.3f y=%.3f z=%.3f yaw=%.1f" % (first[0], first[1], first[2], first[3]))
        print("末点 rel           : x=%.3f y=%.3f z=%.3f yaw=%.1f" % (last[0], last[1], last[2], last[3]))
    if warnings:
        print(">> %d 条 WARNING(见上)" % len(warnings))
    print(">> zigzag_wall_pilot.csv 可直接喂 waypoints_to_qgc_plan.py / make_pilot_viewer.py(用 zigzag_meta.json)")


# ---------------------------------------------------------------------------
def _synthetic_layers():
    """合成 40m宽×60m高 理想平墙: 31 层, x=3, n=(-1,0,0), y∈[-20,20]。"""
    layers = {}
    zs = np.linspace(60.0, 0.0, 31)             # 31 层, 层间距 2.0
    ys = np.linspace(20.0, -20.0, 41)           # 0.5m 沿墙, 左(y=20)->右
    for li, z in enumerate(zs):
        pts = []
        for wi, y in enumerate(ys):
            pts.append([3.0, y, z, yaw_from_normal(-1.0, 0.0), -1.0, 0.0, 0.0])
        layers[li] = np.asarray(pts, dtype=np.float64)
    return layers


def self_test():
    print("=== make_zigzag_wall --self-test ===")
    ok = True
    def check(name, cond, detail=""):
        nonlocal ok
        if not cond:
            ok = False
        print("  [%s] %s %s" % ("PASS" if cond else "FAIL", name, detail))

    meta = {"layer_height": 2.0, "mission_forward_distance": 5.0, "spray_distance": 2.0}
    layers = _synthetic_layers()

    class A:  # vertical 默认
        pattern = "vertical"; column_spacing = None; edge_inset = None
        normal_dot = 0.7; near_x_tol = 3.0
        pilot_corner_angle_deg = 10.0; pilot_max_gap = 10.0; out_dir = None
    groups, pilot, lids, ncol, csp, inset, expx = run(A, meta, layers, lambda m: None)

    # a) 20 列, 每列 31 点
    check("a1: 列数=20", ncol == 20, "got %d" % ncol)
    check("a2: 每列31点", all(len(g) == 31 for g in groups),
          "counts=%s" % sorted(set(len(g) for g in groups)))

    # b) 首点=(3,+19,顶层z=60); 第2列首点 z=最底层 z=0 (弓字反向)
    p0 = groups[0][0]
    check("b1: 首点 x≈3", abs(p0[0] - 3.0) < 1e-6, "x=%.3f" % p0[0])
    check("b2: 首点 y≈+19(左上)", abs(p0[1] - 19.0) < 1e-6, "y=%.3f" % p0[1])
    check("b3: 首点 z≈60(顶层)", abs(p0[2] - 60.0) < 1e-6, "z=%.3f" % p0[2])
    p1 = groups[1][0]
    check("b4: 第2列首点 z≈0(反向从底起)", abs(p1[2] - 0.0) < 1e-6, "z=%.3f" % p1[2])

    # c) 相邻输出点距 ≤ max(层距2, 列距2)+0.01
    lim = max(2.0, csp) + 0.01
    maxjump = 0.0
    flat = np.vstack(groups)
    for i in range(1, len(flat)):
        d = float(np.linalg.norm(flat[i, :3] - flat[i - 1, :3]))
        maxjump = max(maxjump, d)
    check("c: 无大跳变(≤%.2f)" % lim, maxjump <= lim, "maxjump=%.4f" % maxjump)

    # b5) yaw 恒为 0 (法向 -x -> yaw=atan2(0,1)=0)
    check("b5: yaw≈0", abs(p0[3]) < 1e-6, "yaw=%.3f" % p0[3])

    # d) horizontal 行方向逐层交替
    class H:
        pattern = "horizontal"; column_spacing = None; edge_inset = None
        normal_dot = 0.7; near_x_tol = 3.0
        pilot_corner_angle_deg = 10.0; pilot_max_gap = 10.0; out_dir = None
    hgroups, hpilot, hlids, hncol, _, _, _ = run(H, meta, layers, lambda m: None)
    row0, row1 = hgroups[0], hgroups[1]
    check("d1: 行0 左->右(首y≈+20)", abs(row0[0][1] - 20.0) < 1e-6, "y=%.3f" % row0[0][1])
    check("d2: 行1 右->左(首y≈-20)", abs(row1[0][1] - (-20.0)) < 1e-6, "y=%.3f" % row1[0][1])
    check("d3: 行序保留原层号", hlids[:2] == [0, 1], "lids=%s" % hlids[:3])

    print("=== SELF-TEST %s ===" % ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    main()
