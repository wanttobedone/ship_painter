# 喷涂无人机 · 飞手交付现场操作清单

本文档串起三个交付物，给出从"摆机 → 跑规划 → 生成 QGC 任务 → 发给飞手"的完整现场流程。
坐标全部基于 **mission0 相对坐标系**：原点 = 起飞前飞控位置；x = 冻结时机头朝向（水平指向建筑）；
y = 机头左侧；z = 离地高度（米）。

---

## 0. 摆机规则（现场第一步，务必严格）

- 无人机**机头正对目标墙面中点**，放在**建筑前方 `mission_forward_distance` 米处**（当前 `boat2.launch` = 5.0 m，以实际参数为准）。
- 机身保持**水平**，起飞前静置到 EKF/定位稳定。
- 真机为 **RTK 定位**，请确认 RTK 已 Fix。

> 一切换算随 launch 参数自动变化：改了 `mission_forward_distance` / `spray_distance` / `layer_height` 等，
> 重新跑规划即可，导出的 CSV/元数据会自动对应。

---

## 1. 跑规划，拿到导出目录

按既有顺序启动（**不改变任何现有行为**）：

```bash
roslaunch ship_painter boat.launch     # SITL/真机环境
roslaunch ship_painter boat2.launch    # 规划 + 轨迹跟踪节点
```

规划完成后（`reorderLayersByProximity` 之后自动触发），会在 **`export_dir`**（`boat2.launch` 默认
`/tmp/ship_painter_export`）生成 4 个文件：

| 文件 | 用途 |
|---|---|
| `waypoints_full.csv` | 全量航点（离线渲染用，10⁵ 量级） |
| `waypoints_pilot.csv` | 飞手降采样版（生成 QGC 任务用） |
| `export_meta.json` | 部署参数 + 坐标系说明（脚本读取） |
| `README_pilot.txt` | 面向飞手的中文坐标系说明（随件发给飞手） |

节点日志会打印**首个导出层（顶层）首点**与**最低层首点**的相对坐标，便于人工核对。
相关开关：`export_waypoints`（默认 true；设为 false 则行为与改动前完全一致）、
`pilot_spacing`（默认 0.5 m）、`pilot_corner_angle_deg`（默认 15°）。

CSV 列：`layer_index,wp_index,x,y,z,yaw_deg,nx,ny,nz`
（`layer_index=0` 是**顶层**，按飞行顺序排列：先爬到最高层再自上而下喷。）

---

## 2. 生成 QGC `.plan`（真机链路）

现场从 QGC 读取**起飞点经纬度**与**机头航向 heading**（正北=0，顺时针），运行：

```bash
python3.8 tools/waypoints_to_qgc_plan.py \
    --csv /tmp/ship_painter_export/waypoints_pilot.csv \
    --home-lat <纬度> --home-lon <经度> --home-alt 0 \
    --heading <机头方位角deg> \
    --takeoff-alt 2.0 --cruise-speed 1.0 --max-items 300 \
    --out demo.plan
```

- 首次使用建议先自测：`python3.8 tools/waypoints_to_qgc_plan.py --self-test`（全部 PASS 再用）。
- 航点数超 `--max-items` 会**按层均匀抽稀并打印 WARNING**（绝不静默截断）；需要更密就调大 `--max-items`。
- 脚本末尾会打印：航点数、任务总里程、预计航时、最高点高度，以及
  **">> 起飞后首个航点高度 XX m，请飞手确认爬升段无障碍"** —— 这条必须现场逐字确认。
- **默认不含自动 RTL**（`--rtl` 才加）。喷涂结束点可能贴墙，直线返航有撞墙风险，**降落由飞手手动接管**。

### QGC 侧验证（人工）
在 QGC 桌面版打开 `demo.plan`，确认：航点图形的方位与建筑摆放一致（建筑在 heading 方向前方）、
高度剖面合理、首航点在楼顶 → 上传。

> 说明：首个航点在建筑顶层，起飞后会从 `takeoff-alt`(默认 2 m) 斜线爬升到顶层首点。
> 起飞点在墙前 ~5 m、首点在墙前 ~3 m（`forward - spray_distance`），斜线全程在墙外侧，
> 几何上安全；仍请飞手目视确认爬升段无障碍。

---

## 3. 生成离线 3D 查看器（发给飞手直观看路径）

```bash
python3.8 tools/make_pilot_viewer.py \
    --stl models/wall1.stl \
    --csv /tmp/ship_painter_export/waypoints_full.csv \
    --pilot-csv /tmp/ship_painter_export/waypoints_pilot.csv \
    --meta /tmp/ship_painter_export/export_meta.json \
    --target-faces 150000 \
    --out viewer.html
```

- 输出 **单文件、完全离线** 的 `viewer.html`（内联 three.js，~13 MB，Chrome/Edge 双击即开，手机也可）。
- 同目录会另存 `decimated_preview.stl` 供肉眼检查抽稀网格。
- ⚠️ **展示 STL 必须与规划模型一致**：`boat2.launch` 规划用的是 `wall1.stl`，此处就传 `wall1.stl`；
  传错文件会导致路径与建筑错位。
- 查看器功能：建筑[实体/半透明/线框/隐藏]、层滑块（累积 1~N / 仅第 N 层）、彩色层线（顶层红→底层紫）、
  播放动画（1x/5x/20x）、飞手航点悬停显示编号、起飞点标记 + xyz 中文轴、地面网格。

---

## 4. 发给飞手的东西

- `viewer.html`（离线查看器）
- `README_pilot.txt`（坐标系中文说明）
- `demo.plan`（QGC 任务，飞手上传后手动起飞/接管）

---

## 环境备注

- 本机默认 `python3` 为 3.13，**缺 numpy/rospy**；上述脚本请用 **`python3.8`**（含 numpy/open3d）。
- 查看器依赖：`pip install open3d numpy`（抽稀失败时 `pip install trimesh` 兜底读取）。
  three.js 已随仓库放在 `tools/vendor/`，离线可用；缺失时脚本会从 CDN 下载并缓存。

## 红线（务必遵守）
- 未改任何现有控制/规划/B样条/MPC 逻辑与话题；导出功能异常只告警、不中断飞行。
- QGC 任务默认不自动返航，喷涂结束由飞手手动接管降落。
- 所有坐标/方位换算均带自测断言，`--self-test` 通过后再上真机。
