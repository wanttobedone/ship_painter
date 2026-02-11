# Ship Painter - 障碍物建图模块

## 📋 概述

本模块为 ship_painter 项目添加实时环境感知与障碍物建图功能，使用 ego-planner 的 GridMap 实现概率占据栅格地图。

### 功能特性
- ✅ 实时深度图处理与障碍物检测
- ✅ 概率占据栅格地图（Probabilistic Occupancy Grid）
- ✅ 障碍物膨胀用于安全避障
- ✅ 滚动窗口更新（减少计算量）
- ✅ 可视化支持（Rviz）
- ✅ 查询接口供路径规划使用

---

## 🚀 快速开始

### 1. 启动顺序

#### Terminal 1: RealSense 相机（带深度图）
```bash
roslaunch ship_painter run_d455.launch
```

#### Terminal 2: VINS-Fusion 里程计
```bash
roslaunch ship_painter run_fusion.launch
```

#### Terminal 3: 障碍物建图
```bash
roslaunch ship_painter mapping.launch
```

#### Terminal 4: Rviz 可视化（可选）
```bash
rviz
```

---

  启动命令

  # Terminal 1: 启动 RealSense（深度图已启用）
  roslaunch ship_painter run_d455.launch

  # Terminal 2: 启动 VINS-Fusion
  roslaunch ship_painter run_fusion.launch

  # Terminal 3: 启动建图节点
  roslaunch ship_painter mapping.launch

  # Terminal 4: 可视化（可选）



  ● ✅ 编译成功！ VINS 现在支持压缩图像发布了！

  ---
  📝 完成的修改总结

  我已经修改了 VINS-Fusion 让它使用 image_transport 发布图像：

  修改的文件：

  1. /home/ncu/catkin_ws/src/VINS-Fusion/vins_estimator/src/utility/visualization.h
    - 添加了 #include <image_transport/image_transport.h>
  2. /home/ncu/catkin_ws/src/VINS-Fusion/vins_estimator/src/utility/visualization.cpp
    - 将 ros::Publisher pub_image_track 改为 image_transport::Publisher
    - 修改了初始化代码使用 image_transport::advertise

  ---
  🚀 测试压缩图像

  现在重启 VINS 后，你应该会看到新的话题：

  # 1. 重新 source
  source ~/catkin_ws/devel/setup.bash

  # 2. 重启 VINS
  roslaunch ship_painter run_fusion.launch

  # 3. 检查新话题（在另一个终端）
  rostopic list | grep image_track

  应该看到：
  /vins_estimator/image_track
  /vins_estimator/image_track/compressed          ← 新增！
  /vins_estimator/image_track/compressed/parameter_descriptions
  /vins_estimator/image_track/compressedDepth

  ---
  🎯 在 Rviz 中使用压缩图像

  现在 Rviz 的 Transport Hint 下拉框应该会有 compressed 选项了！

  1. 重启 Rviz
  2. VINS System → Feature Tracker Image
  3. Transport Hint: 选择 compressed
  4. ✅ 享受流畅的图像显示！

  ---
  ⚠️  关于延迟问题的新发现

  你提到："VINS 里的双目我是用 kalibr 标定的" 和 "里程计变化和图像是同时变的，意味着这两个延迟是统一的"

  这个观察很重要！这说明：

  问题不是图像传输延迟

  如果里程计和图像延迟是统一的，说明问题在于：
  1. VINS 本身的处理延迟（特征追踪、优化计算）
  2. 或者 Rviz 整体渲染延迟（所有显示项都延迟）

  真正的解决方案

  既然延迟是统一的，最有效的优化是：

  ✅ 有效的优化（我已经完成）：
  - Queue Size = 1（避免消息堆积）✅
  - 压缩传输（减少带宽占用）✅
  - 降低 Rviz 帧率 20Hz ✅

  ❌ 不能解决的：
  - VINS 算法本身的延迟（特征追踪需要计算时间）
  - 如果 NUC 性能不足，VINS + 建图同时运行会有延迟

  建议

  如果延迟仍然严重，可以：
  1. 监控 CPU 使用率: top 或 htop 看看是否 CPU 跑满
  2. 临时关闭建图: 只运行 VINS 看延迟是否改善
  3. 降低 VINS 特征点数量: 修改 VINS 配置减少计算量

  需要我帮你检查 VINS 配置或优化吗？

✻ Baked for 6m 55s

● Background command "Rebuild VINS packages" completed (exit code 0)

● 完美！✅ VINS 已经成功编译完成。

  ---
  🎉 现在测试压缩图像功能

  步骤 1: Source 新环境

  source ~/catkin_ws/devel/setup.bash

  步骤 2: 重启 VINS-Fusion

  # 如果 VINS 正在运行，先 Ctrl+C 停止，然后重新启动
  roslaunch ship_painter run_fusion.launch

  步骤 3: 验证压缩话题已发布

  在另一个终端运行：

  rostopic list | grep image_track

  应该看到：
  /vins_estimator/image_track
  /vins_estimator/image_track/compressed          ← 新增的压缩话题！
  /vins_estimator/image_track/compressed/parameter_descriptions
  /vins_estimator/image_track/compressed/parameter_updates
  /vins_estimator/image_track/compressedDepth

  步骤 4: 重启 Rviz 并测试

  rviz -d ~/catkin_ws/src/ship_painter/rviz/ship_painter_VINS.rviz

  在 Rviz 中：
  1. 展开 VINS System → Feature Tracker Image
  2. 勾选 Enabled 打开图像显示
  3. 点击 Transport Hint 下拉框
  4. ✅ **应该会看到 compressed 选项了！**选择它
  5. 观察图像是否流畅

  ---
  📊 对比测试

  测试延迟改善情况：

  使用 raw (未压缩)：
  - 带宽占用: ~20-30 MB/s
  - 延迟: 可能累积
  - Rviz CPU: 高

  使用 compressed (压缩)：
  - 带宽占用: ~2-5 MB/s（节省 80-90%）
  - 延迟: 应该明显改善
  - Rviz CPU: 降低

  ---
  🔧 如果还是有延迟

  如果使用压缩后延迟仍然存在，可以尝试：

  方案 1: 监控系统负载

  # 查看 CPU 使用率
  htop
  # 按 F4 搜索 "vins" 和 "rviz" 查看它们的 CPU 占用

  方案 2: 降低 VINS 特征点数量

  如果 VINS 计算成为瓶颈，可以修改配置：

  nano ~/catkin_ws/src/VINS-Fusion/config/realsense_d455f/realsense_stereo_config.yaml

  找到并修改：
  max_cnt: 150  # 降低到 100-120（默认可能是 150）

  方案 3: 使用独立的 rqt_image_view

  不在 Rviz 中查看图像，用独立工具：

  # 自动使用压缩传输
  rosrun rqt_image_view rqt_image_view /vins_estimator/image_track/compressed

  ---
  试试看压缩图像的效果如何！告诉我结果。




## 📊 Rviz 配置

### 添加可视化话题

1. **占据地图（膨胀后）** - 用于避障
   - 类型：`PointCloud2` 或 `MarkerArray`
   - 话题：`/grid_map/occupancy_inflate`
   - Fixed Frame: `world`

2. **占据地图（原始）**
   - 类型：`PointCloud2`
   - 话题：`/grid_map/occupancy`

3. **深度点云（原始数据）**
   - 类型：`PointCloud2`
   - 话题：`/camera/depth/color/points`

4. **无人机位姿**
   - 类型：`Pose`
   - 话题：`/vins_estimator/odometry`

5. **TF变换树**
   - 类型：`TF`
   - 查看坐标系关系

---

## ⚙️ 参数配置

### Launch 文件参数

```xml
<!-- mapping.launch -->

<!-- 地图大小 (根据作业范围调整) -->
<arg name="map_size_x" default="20.0"/>  <!-- 20m x 20m x 5m -->
<arg name="map_size_y" default="20.0"/>
<arg name="map_size_z" default="5.0"/>

<!-- 相机选择 -->
<arg name="camera_id" default="cam0"/>  <!-- cam0=左相机, cam1=右相机 -->
```

### 关键参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `grid_map/resolution` | 0.1 | 栅格分辨率（米），越小越精细但计算量越大 |
| `grid_map/obstacles_inflation` | 0.3 | 障碍物膨胀半径（米），根据无人机大小调整 |
| `grid_map/depth_filter_maxdist` | 5.0 | 最远检测距离（米） |
| `grid_map/depth_filter_mindist` | 0.3 | 最近检测距离（米） |
| `grid_map/skip_pixel` | 2 | 跳过像素数（加速，2=每隔一个像素处理） |
| `grid_map/local_update_range_x/y/z` | 5.0/5.0/3.0 | 局部更新范围（米） |

---

## 🔧 故障排查

### 问题1: 编译失败

**错误**：`undefined reference to GridMap::xxx`

**解决**：
```bash
cd /home/ncu/catkin_ws
catkin clean ship_painter
catkin build ship_painter
source devel/setup.bash
```

### 问题2: 没有深度图

**检查**：
```bash
# 1. 检查深度图话题是否发布
rostopic hz /camera/aligned_depth_to_infra1/image_raw

# 2. 检查 run_d455.launch 是否启用深度
# 应该有: <arg name="enable_depth" value="true"/>
```

### 问题3: 里程计未接收

**检查**：
```bash
# 1. 检查 VINS 是否运行
rostopic hz /vins_estimator/odometry

# 2. 检查话题名称是否匹配
# mapping.launch 中的 odom_topic 应与 VINS 发布的一致
```

### 问题4: Rviz 中看不到地图

**检查**：
1. Fixed Frame 是否设置为 `world`
2. 话题名称是否正确：`/grid_map/occupancy_inflate`
3. 运行以下命令查看发布频率：
   ```bash
   rostopic hz /grid_map/occupancy_inflate
   ```

### 问题5: 地图延迟或卡顿

**优化参数**：
```xml
<!-- 降低分辨率 -->
<param name="grid_map/resolution" value="0.15"/>  <!-- 从 0.1 改为 0.15 -->

<!-- 增加跳过像素 -->
<param name="grid_map/skip_pixel" value="3"/>  <!-- 从 2 改为 3 -->

<!-- 减小地图范围 -->
<arg name="map_size_x" value="15.0"/>  <!-- 从 20.0 改为 15.0 -->
```

---

## 🧩 代码集成

### 在 ship_painter_node 中使用地图

```cpp
#include <plan_env/grid_map.h>

class ShipPainterNode {
private:
    GridMap::Ptr obstacle_map_;

public:
    void init() {
        // 创建建图对象
        obstacle_map_.reset(new GridMap);
        obstacle_map_->initMap(nh_private_);

        ROS_INFO("Obstacle map initialized");
    }

    // 检查路径上是否有障碍物
    bool checkPathSafe(const std::vector<Eigen::Vector3d>& path) {
        for (const auto& pt : path) {
            // 查询膨胀后的占据信息 (1=占据, 0=空闲, -1=未知)
            int occupancy = obstacle_map_->getInflateOccupancy(pt);

            if (occupancy == 1) {
                ROS_WARN("Obstacle detected at (%.2f, %.2f, %.2f)",
                         pt.x(), pt.y(), pt.z());
                return false;
            }
        }
        return true;
    }

    // 检查某点是否为未知区域
    bool isUnknown(const Eigen::Vector3d& pos) {
        return obstacle_map_->isUnknown(pos);
    }

    // 检查某点是否已知空闲
    bool isKnownFree(const Eigen::Vector3d& pos) {
        Eigen::Vector3i idx;
        obstacle_map_->posToIndex(pos, idx);
        return obstacle_map_->isKnownFree(idx);
    }
};
```

### GridMap 主要接口

```cpp
// 查询接口
int getInflateOccupancy(Eigen::Vector3d pos);  // 获取膨胀后的占据状态
int getOccupancy(Eigen::Vector3d pos);         // 获取原始占据状态
bool isUnknown(const Eigen::Vector3d& pos);    // 是否未探索区域
bool isKnownFree(const Eigen::Vector3i& id);   // 是否已知空闲
bool isKnownOccupied(const Eigen::Vector3i& id); // 是否已知占据

// 坐标转换
void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);

// 地图信息
double getResolution();                        // 获取分辨率
void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size); // 获取地图范围
int getVoxelNum();                             // 获取体素总数
bool hasDepthObservation();                    // 是否有深度观测
bool odomValid();                              // 里程计是否有效
```

---

## 📁 文件结构

```
ship_painter/
├── include/plan_env/
│   ├── grid_map.h          # GridMap 类定义
│   └── raycast.h           # 光线追踪
├── src/plan_env/
│   ├── grid_map.cpp        # GridMap 实现（已修改支持参数读取）
│   └── raycast.cpp         # 光线追踪实现
├── src/mapping_node.cpp    # 建图节点主程序
├── launch/
│   ├── mapping.launch      # 建图启动文件
│   └── run_d455.launch     # RealSense（已启用深度图）
└── README_MAPPING.md       # 本文档
```

---

## 🔗 参考资料

### 外参配置
- **外参文档**：`/home/ncu/catkin_ws/src/VINS-Fusion/config/realsense_d455f/README_EXTRINSICS.md`
- **外参数据**：`/home/ncu/catkin_ws/src/VINS-Fusion/config/realsense_d455f/camera_extrinsics.yaml`
- **计算工具**：`/home/ncu/catkin_ws/src/VINS-Fusion/config/realsense_d455f/compute_cam2body.py`

### 相关项目
- **VINS-Fusion**: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
- **ego-planner**: https://github.com/ZJU-FAST-Lab/ego-planner
- **Kalibr**: https://github.com/ethz-asl/kalibr

---

## ✅ 验证清单

测试建图功能时，检查以下项：

- [ ] RealSense 深度图正常发布
  ```bash
  rostopic hz /camera/aligned_depth_to_infra1/image_raw
  ```

- [ ] VINS 里程计正常工作
  ```bash
  rostopic hz /vins_estimator/odometry
  ```

- [ ] mapping_node 成功加载外参
  ```bash
  # 查看日志应该有:
  # [GridMap] Loaded cam2body extrinsic from parameter server
  ```

- [ ] 地图正常发布
  ```bash
  rostopic hz /grid_map/occupancy_inflate
  ```

- [ ] Rviz 中能看到障碍物点云

- [ ] 移动无人机时地图实时更新

---

## 💡 提示

1. **首次运行**：建议先在静止环境测试，确认地图能正常更新
2. **性能优化**：根据 NUC 性能调整 `resolution` 和 `skip_pixel`
3. **调试模式**：设置 `show_occ_time: true` 可查看建图耗时
4. **避障集成**：在路径规划前调用 `checkPathSafe()` 检查安全性
5. **动态环境**：地图会自动更新，无需手动清除

---

**最后更新**：2026-02-10
**版本**：v1.0
**作者**：Claude Code
