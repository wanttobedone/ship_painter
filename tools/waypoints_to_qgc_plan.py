#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoints_to_qgc_plan.py  —  把 ship_painter 导出的 mission0 相对坐标航点(CSV)
                              转换为 QGroundControl 的 .plan 任务文件(QGC → PX4 真机链路)。

用途
----
交付物A(ship_painter_node 的 exportWaypoints)会导出 waypoints_pilot.csv（飞手降采样版），
坐标系为 mission0 相对系: x=前(机头初始朝向,指向建筑), y=左, z=离地高度(m), yaw_deg=相对机头初始朝向。
本脚本把这些相对坐标结合“现场起飞点经纬度 + 机头朝向方位角”换算成经纬度航点, 生成 .plan,
飞手用 QGC 打开/上传即可让 PX4(RTK 定位)执行 demo 航线。

依赖
----
仅标准库 (json / csv / math / argparse)，任何 Python3 均可运行。

现场用法
--------
  1) 按摆机规则把无人机放好: 机头正对目标墙面中点, 建筑前方 mission_forward_distance 米处, 机身水平。
  2) 从 QGC 读取当前起飞点经纬度(home-lat/home-lon)与机头航向(heading, 指南针/QGC 航向读数)。
  3) 运行:
       python3 waypoints_to_qgc_plan.py \
           --csv waypoints_pilot.csv \
           --home-lat 30.123456 --home-lon 114.123456 --home-alt 0 \
           --heading 90 \
           --takeoff-alt 2.0 --cruise-speed 1.0 --max-items 300 \
           --out demo.plan
  4) QGC 打开 demo.plan, 核对航点图形与建筑方位一致后上传。

自测:
       python3 waypoints_to_qgc_plan.py --self-test

坐标换算 (mission0 相对 → ENU 偏移 → 经纬度)
-------------------------------------------
  theta = heading(度→弧度), 方位角: 正北=0, 顺时针为正
  dE = x*sin(theta) - y*cos(theta)     # 东向偏移(m)
  dN = x*cos(theta) + y*sin(theta)     # 北向偏移(m)
  lat = home_lat + (dN / R) * 180/pi
  lon = home_lon + (dE / (R*cos(home_lat_rad))) * 180/pi   ; R = 6378137 (WGS84 长半轴)
  航点高度 alt = z (离地高, QGC 相对高度帧, Home 处地面 = 0)
  航点航向(方位角) yaw_plan = wrap360(heading + (-yaw_rel_deg))
    ——mission0 的 yaw 逆时针为正(ROS/ENU 惯例), 方位角顺时针为正, 故相减。

安全说明
--------
- 默认不含自动 RTL(--rtl 才加)。喷涂结束点可能贴墙, 直线返航有撞墙风险, 请飞手手动接管降落。
- 首个航点在建筑顶层, 起飞后会从 takeoff-alt 斜线爬升到顶层首点; 脚本会打印该爬升高度, 飞手需确认爬升段无障碍。
- 航点数超过 --max-items 时按层均匀抽稀(保每层首末点)并打印 WARNING, 绝不静默截断。
"""

import argparse
import csv
import json
import math
import sys
from collections import OrderedDict

R_EARTH = 6378137.0  # WGS84 长半轴 (m)

# MAVLink 命令码
CMD_NAV_WAYPOINT = 16
CMD_NAV_RETURN_TO_LAUNCH = 20
CMD_NAV_TAKEOFF = 22
CMD_DO_CHANGE_SPEED = 178

FRAME_GLOBAL_REL_ALT = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
FRAME_MISSION = 2         # MAV_FRAME_MISSION (用于 DO_* / RTL 命令项)


# ---------------------------------------------------------------------------
# 基础换算
# ---------------------------------------------------------------------------
def wrap360(deg):
    """归一化到 [0, 360)。"""
    d = math.fmod(deg, 360.0)
    if d < 0:
        d += 360.0
    return d


def local_to_enu(x, y, heading_deg):
    """mission0 相对坐标(x=前,y=左) → ENU 水平偏移(dE, dN), 单位 m。"""
    theta = math.radians(heading_deg)
    dE = x * math.sin(theta) - y * math.cos(theta)
    dN = x * math.cos(theta) + y * math.sin(theta)
    return dE, dN


def enu_to_latlon(dE, dN, home_lat, home_lon):
    """ENU 偏移(m) → 经纬度(局部切平面近似)。"""
    lat = home_lat + math.degrees(dN / R_EARTH)
    lon = home_lon + math.degrees(dE / (R_EARTH * math.cos(math.radians(home_lat))))
    return lat, lon


def local_to_latlon(x, y, home_lat, home_lon, heading_deg):
    dE, dN = local_to_enu(x, y, heading_deg)
    lat, lon = enu_to_latlon(dE, dN, home_lat, home_lon)
    return lat, lon


def haversine(lat1, lon1, lat2, lon2):
    """两经纬度间大圆距离(m), 用于自测校验换算距离。"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
    return 2 * R_EARTH * math.asin(math.sqrt(a))


# ---------------------------------------------------------------------------
# CSV 读取与抽稀
# ---------------------------------------------------------------------------
def read_waypoints(csv_path):
    """读取 waypoints_*.csv, 返回 [dict(layer,x,y,z,yaw_deg), ...] 保持行序(飞行顺序)。"""
    rows = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        need = {"layer_index", "x", "y", "z", "yaw_deg"}
        if not need.issubset(set(reader.fieldnames or [])):
            raise ValueError("CSV 缺少必要列, 需要至少 %s, 实际 %s"
                             % (sorted(need), reader.fieldnames))
        for r in reader:
            rows.append({
                "layer": int(float(r["layer_index"])),
                "x": float(r["x"]),
                "y": float(r["y"]),
                "z": float(r["z"]),
                "yaw_deg": float(r["yaw_deg"]),
            })
    return rows


def _decimate(layers, target_total, total):
    """按层均匀抽稀, 每层保首末点。target_total 为期望总点数。"""
    out = []
    for pts in layers.values():
        k = len(pts)
        if k <= 2:
            out.extend(pts)
            continue
        keep_n = max(2, min(k, int(round(k * target_total / total))))
        if keep_n >= k:
            out.extend(pts)
        else:
            idxs = sorted(set(int(round(i * (k - 1) / (keep_n - 1)))
                              for i in range(keep_n)))
            out.extend(pts[i] for i in idxs)
    return out


def downsample_rows(rows, allowed_wp):
    """
    若航点数 > allowed_wp, 按层均匀抽稀(保每层首末点)。
    返回 (rows_out, was_downsampled)。绝不静默截断。
    """
    if len(rows) <= allowed_wp:
        return rows, False

    layers = OrderedDict()
    for r in rows:
        layers.setdefault(r["layer"], []).append(r)

    total = len(rows)
    n_layers = len(layers)
    target = allowed_wp
    out = _decimate(layers, target, total)
    # per-layer 最少 2 点会导致下限 = 2*层数; 逐步收紧直到不超上限
    while len(out) > allowed_wp and target > 2 * n_layers:
        target = int(target * 0.95) or 1
        out = _decimate(layers, target, total)
    return out, True


# ---------------------------------------------------------------------------
# .plan 构建
# ---------------------------------------------------------------------------
def make_simple_item(do_jump_id, command, frame, params, altitude=0.0, alt_mode=1):
    return {
        "AMSLAltAboveTerrain": None,
        "Altitude": altitude,
        "AltitudeMode": alt_mode,
        "autoContinue": True,
        "command": command,
        "doJumpId": do_jump_id,
        "frame": frame,
        "params": params,
        "type": "SimpleItem",
    }


def build_plan(rows, home_lat, home_lon, home_alt, heading,
               takeoff_alt, cruise_speed, add_speed_item, add_rtl):
    """构建 QGC .plan dict, 返回 (plan_dict, stats_dict)。"""
    items = []
    jid = 1

    # item0: 起飞 (yaw=NaN → JSON null, 表示不指定起飞航向)
    items.append(make_simple_item(
        jid, CMD_NAV_TAKEOFF, FRAME_GLOBAL_REL_ALT,
        [0.0, 0.0, 0.0, None, home_lat, home_lon, takeoff_alt],
        altitude=takeoff_alt))
    jid += 1

    # item1(可选): 巡航速度
    if add_speed_item:
        items.append(make_simple_item(
            jid, CMD_DO_CHANGE_SPEED, FRAME_MISSION,
            [1.0, float(cruise_speed), -1.0, 0.0, 0.0, 0.0, 0.0],
            altitude=0.0))
        jid += 1

    # 航点
    wp_latlon = []
    for r in rows:
        lat, lon = local_to_latlon(r["x"], r["y"], home_lat, home_lon, heading)
        alt = r["z"]
        yaw_plan = wrap360(heading + (-r["yaw_deg"]))
        items.append(make_simple_item(
            jid, CMD_NAV_WAYPOINT, FRAME_GLOBAL_REL_ALT,
            [0.0, 0.3, 0.0, yaw_plan, lat, lon, alt],
            altitude=alt))
        jid += 1
        wp_latlon.append((lat, lon, alt))

    # 末尾(可选): 返航
    if add_rtl:
        items.append(make_simple_item(
            jid, CMD_NAV_RETURN_TO_LAUNCH, FRAME_MISSION,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            altitude=0.0))
        jid += 1

    plan = {
        "fileType": "Plan",
        "groundStation": "QGroundControl",
        "version": 1,
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "rallyPoints": {"points": [], "version": 2},
        "mission": {
            "version": 2,
            "firmwareType": 12,   # PX4
            "vehicleType": 2,     # 多旋翼
            "cruiseSpeed": float(cruise_speed),
            "hoverSpeed": float(cruise_speed),
            "plannedHomePosition": [home_lat, home_lon, home_alt],
            "globalPlanAltitudeMode": 1,
            "items": items,
        },
    }

    # 统计(基于本地相对坐标, 与实际里程一致)
    dist = 0.0
    for i in range(1, len(rows)):
        a, b = rows[i - 1], rows[i]
        dist += math.sqrt((b["x"] - a["x"])**2 + (b["y"] - a["y"])**2 + (b["z"] - a["z"])**2)
    max_alt = max([takeoff_alt] + [r["z"] for r in rows]) if rows else takeoff_alt
    first_wp_alt = rows[0]["z"] if rows else takeoff_alt
    stats = {
        "num_waypoints": len(rows),
        "total_distance_m": dist,
        "flight_time_s": dist / cruise_speed if cruise_speed > 0 else float("inf"),
        "max_alt_m": max_alt,
        "first_wp_alt_m": first_wp_alt,
    }
    return plan, stats


# ---------------------------------------------------------------------------
# 自测
# ---------------------------------------------------------------------------
def self_test():
    print("=== waypoints_to_qgc_plan --self-test ===")
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        status = "PASS" if cond else "FAIL"
        if not cond:
            ok = False
        print("  [%s] %s %s" % (status, name, detail))

    home_lat, home_lon = 30.5, 114.3

    # 断言1: y=0, yaw_rel=0 的航点应正好在 home 正前方 heading 方向, yaw_plan=heading
    for heading in (0.0, 45.0, 90.0, 180.0, 270.0):
        lat, lon = local_to_latlon(10.0, 0.0, home_lat, home_lon, heading)
        # 反算方位角
        dE, dN = local_to_enu(10.0, 0.0, heading)
        bearing = wrap360(math.degrees(math.atan2(dE, dN)))
        yaw_plan = wrap360(heading + (-0.0))
        check("forward bearing @heading=%g" % heading,
              abs((bearing - heading + 180) % 360 - 180) < 1e-6,
              "bearing=%.3f" % bearing)
        check("yaw_plan==heading @%g" % heading,
              abs((yaw_plan - heading + 180) % 360 - 180) < 1e-6,
              "yaw_plan=%.3f" % yaw_plan)

    # 断言2: heading=0 时四个基本方向的符号 (x=前=北, y=左=西)
    cases = [
        ("x=+10 -> 正北(lat+)", 10.0, 0.0, lambda la, lo: la > home_lat and abs(lo - home_lon) < 1e-9),
        ("y=+10 -> 正西(lon-)", 0.0, 10.0, lambda la, lo: lo < home_lon and abs(la - home_lat) < 1e-9),
        ("x=-10 -> 正南(lat-)", -10.0, 0.0, lambda la, lo: la < home_lat and abs(lo - home_lon) < 1e-9),
        ("y=-10 -> 正东(lon+)", 0.0, -10.0, lambda la, lo: lo > home_lon and abs(la - home_lat) < 1e-9),
    ]
    for name, x, y, pred in cases:
        lat, lon = local_to_latlon(x, y, home_lat, home_lon, 0.0)
        check("dir sign: " + name, pred(lat, lon),
              "lat=%.7f lon=%.7f" % (lat, lon))

    # 断言3: 换算距离误差 < 0.1% (haversine vs 平面距离), 多个已知点/航向
    test_pts = [(10, 0, 5), (0, 10, 5), (-10, 0, 5), (0, -10, 5), (30, -20, 12)]
    for heading in (0.0, 37.0, 123.0, 250.0):
        for (x, y, z) in test_pts:
            lat, lon = local_to_latlon(x, y, home_lat, home_lon, heading)
            d_planar = math.sqrt(x * x + y * y)
            d_geo = haversine(home_lat, home_lon, lat, lon)
            rel_err = abs(d_geo - d_planar) / max(d_planar, 1e-9)
            check("dist err<0.1%% (h=%g,x=%g,y=%g)" % (heading, x, y),
                  rel_err < 1e-3, "err=%.4g%%" % (rel_err * 100))

    # 断言4: 生成的 .plan 能被 json.load 解析且结构正确
    rows = [{"layer": 0, "x": 10, "y": 0, "z": 5, "yaw_deg": 0.0},
            {"layer": 0, "x": 0, "y": 10, "z": 5, "yaw_deg": 90.0}]
    plan, stats = build_plan(rows, home_lat, home_lon, 0.0, 90.0,
                             2.0, 1.0, add_speed_item=True, add_rtl=False)
    s = json.dumps(plan)
    reparsed = json.loads(s)
    check("plan json roundtrip", reparsed["fileType"] == "Plan")
    check("takeoff is first item", reparsed["mission"]["items"][0]["command"] == CMD_NAV_TAKEOFF)
    check("doJumpId strictly increasing",
          [it["doJumpId"] for it in reparsed["mission"]["items"]] ==
          list(range(1, len(reparsed["mission"]["items"]) + 1)))

    print("=== SELF-TEST %s ===" % ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="把 ship_painter 相对坐标航点(CSV)转成 QGC .plan 文件",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--csv", help="输入 waypoints_pilot.csv (mission0 相对坐标)")
    ap.add_argument("--home-lat", type=float, help="起飞点纬度 (deg)")
    ap.add_argument("--home-lon", type=float, help="起飞点经度 (deg)")
    ap.add_argument("--home-alt", type=float, default=0.0, help="起飞点高度 (m, AMSL, 可默认0)")
    ap.add_argument("--heading", type=float, help="机头朝向方位角 (deg, 正北=0 顺时针)")
    ap.add_argument("--takeoff-alt", type=float, default=2.0, help="起飞悬停高度 (m)")
    ap.add_argument("--cruise-speed", type=float, default=1.0, help="巡航速度 (m/s)")
    ap.add_argument("--max-items", type=int, default=300, help="任务项上限(含起飞/变速/RTL)")
    ap.add_argument("--rtl", action="store_true", help="末尾追加自动返航 (默认关闭, 建议飞手手动降落)")
    ap.add_argument("--no-speed-item", action="store_true", help="不写 DO_CHANGE_SPEED 项")
    ap.add_argument("--out", help="输出 .plan 路径")
    ap.add_argument("--self-test", action="store_true", help="运行内置自测并退出")
    args = ap.parse_args()

    if args.self_test:
        sys.exit(self_test())

    # 参数校验
    missing = [k for k in ("csv", "home_lat", "home_lon", "heading", "out")
               if getattr(args, k) is None]
    if missing:
        ap.error("缺少必要参数: %s (或用 --self-test)" % ", ".join("--" + m.replace("_", "-") for m in missing))

    rows = read_waypoints(args.csv)
    if not rows:
        ap.error("CSV 中没有航点")

    add_speed_item = not args.no_speed_item
    overhead = 1 + (1 if add_speed_item else 0) + (1 if args.rtl else 0)  # 起飞 + 变速 + RTL
    allowed_wp = max(2, args.max_items - overhead)

    rows_use, downsampled = downsample_rows(rows, allowed_wp)
    if downsampled:
        print("WARNING: 航点数 %d > 上限对应的 %d, 已按层均匀抽稀(保每层首末点)至 %d 点。"
              % (len(rows), allowed_wp, len(rows_use)))
        print("         如需保留更多航点请增大 --max-items。")

    plan, stats = build_plan(
        rows_use, args.home_lat, args.home_lon, args.home_alt, args.heading,
        args.takeoff_alt, args.cruise_speed, add_speed_item, args.rtl)

    with open(args.out, "w") as f:
        json.dump(plan, f, indent=4)
        f.write("\n")

    total_items = len(plan["mission"]["items"])
    print("已写出 %s" % args.out)
    print("--------------------------------------------------")
    print("任务项总数         : %d (起飞1 + 变速%d + 航点%d + RTL%d)"
          % (total_items, 1 if add_speed_item else 0, stats["num_waypoints"], 1 if args.rtl else 0))
    print("航点数             : %d" % stats["num_waypoints"])
    print("任务总里程         : %.1f m" % stats["total_distance_m"])
    print("预计航时(里程/巡航): %.0f s (%.1f min) @ %.2f m/s"
          % (stats["flight_time_s"], stats["flight_time_s"] / 60.0, args.cruise_speed))
    print("最高点高度         : %.2f m (离地)" % stats["max_alt_m"])
    print("--------------------------------------------------")
    print(">> 起飞后首个航点高度 %.2f m，请飞手确认爬升段无障碍。" % stats["first_wp_alt_m"])
    if not args.rtl:
        print(">> 未含自动 RTL：喷涂结束点可能贴墙，请飞手手动接管降落。")
    print(">> 请在 QGC 中打开 .plan，核对航点图形方位与建筑摆放一致后再上传。")


if __name__ == "__main__":
    main()
