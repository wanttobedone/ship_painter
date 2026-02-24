/**
 *  障碍物建图节点 - 使用 GridMap 进行实时环境感知
 *   - 订阅 RealSense 深度图和 VINS 里程计
 *   - 使用概率占据栅格地图进行建图
 *   - 发布占据地图和膨胀地图用于避障
 *   - 提供查询接口供路径规划使用
 *
 * 话题订阅：
 *   - /camera/aligned_depth_to_infra1/image_raw (深度图)
 *   - /vins_estimator/odometry (里程计)
 *
 * 话题发布：
 *   - /grid_map/occupancy (原始占据地图)
 *   - /grid_map/occupancy_inflate (膨胀后的占据地图)
 *
 * 参数配置：
 *   - grid_map/cam2body (相机外参，4x4矩阵)
 *   - grid_map/cx, cy, fx, fy (相机内参)
 *   - grid_map/map_size_x/y/z (地图大小)
 *   - 更多参数见 launch 文件
 *
 */

#include <ros/ros.h>
#include <plan_env/grid_map.h>
#include <signal.h>

// 全局变量用于退出
GridMap::Ptr g_grid_map;

/**
 * 信号处理函数，用于Ctrl+C退出
 */
void signalHandler(int sig) {
    ROS_INFO("[MappingNode] Shutting down gracefully...");
    if (g_grid_map) {
        ROS_INFO("[MappingNode] Publishing final map...");
        g_grid_map->publishMap();
        g_grid_map->publishMapInflate(true);
    }
    ros::shutdown();
}

/**
 * @brief 主函数
 */
int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "mapping_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // 注册信号处理
    signal(SIGINT, signalHandler);

     
    ROS_INFO("Ship Painter - Obstacle Mapping Node");
    ROS_INFO(" 实时环境感知与障碍物建图");
    ROS_INFO("基于ego-planner GridMap (概率占据栅格地图)");
     

    // 创建建图对象
    g_grid_map.reset(new GridMap);

    // 初始化地图（会从参数服务器读取所有配置）
    ROS_INFO("[MappingNode] Initializing GridMap...");
    try {
        g_grid_map->initMap(nh_private);
        ROS_INFO("[MappingNode] GridMap initialized successfully");
    } catch (const std::exception& e) {
        ROS_ERROR("[MappingNode] Failed to initialize GridMap: %s", e.what());
        return 1;
    }

    // 打印配置信息
    Eigen::Vector3d map_origin, map_size;
    g_grid_map->getRegion(map_origin, map_size);
    double resolution = g_grid_map->getResolution();

     
    ROS_INFO("地图配置:");
    ROS_INFO("  原点: [%.2f, %.2f, %.2f]", map_origin.x(), map_origin.y(), map_origin.z());
    ROS_INFO("  大小: [%.2f, %.2f, %.2f] m", map_size.x(), map_size.y(), map_size.z());
    ROS_INFO("  分辨率: %.2f m (%.0f cm)", resolution, resolution * 100);
    ROS_INFO("  体素数量: %d", g_grid_map->getVoxelNum());
     

    // 等待传感器数据
    ROS_INFO("[MappingNode] Waiting for sensor data...");
    ROS_INFO("  Expected topics:");

    // 从参数服务器获取话题名称
    std::string odom_topic, depth_topic;
    nh_private.param<std::string>("grid_map/odom_topic", odom_topic, "/vins_estimator/odometry");
    nh_private.param<std::string>("grid_map/depth_topic", depth_topic, "/camera/aligned_depth_to_infra1/image_raw");

    ROS_INFO("    Odometry: %s", odom_topic.c_str());
    ROS_INFO("    Depth:    %s", depth_topic.c_str());
    ROS_INFO("");
    ROS_INFO("  Tip:使用以下命令检查话题是否发布:");
    ROS_INFO("    rostopic hz %s", odom_topic.c_str());
    ROS_INFO("    rostopic hz %s", depth_topic.c_str());
     

    // 等待数据就绪
    ros::Rate wait_rate(1);
    int wait_count = 0;
    while (ros::ok() && !g_grid_map->odomValid() && wait_count < 30) {
        ros::spinOnce();
        wait_rate.sleep();
        wait_count++;

        if (wait_count % 5 == 0) {
            ROS_INFO("[MappingNode] Still waiting for odometry... (%d/30s)", wait_count);
        }
    }

    if (!g_grid_map->odomValid()) {
        ROS_ERROR("[MappingNode] Timeout waiting for odometry!");
        ROS_ERROR("  请检查:");
        ROS_ERROR("    1. VINS-Fusion 是否正常运行?");
        ROS_ERROR("    2. 话题名称是否正确?");
        ROS_ERROR("    3. 使用 'rostopic list' 查看可用话题");
        return 1;
    }

    ROS_INFO("[MappingNode] Odometry received");

    // 等待深度数据
    wait_count = 0;
    while (ros::ok() && !g_grid_map->hasDepthObservation() && wait_count < 30) {
        ros::spinOnce();
        wait_rate.sleep();
        wait_count++;

        if (wait_count % 5 == 0) {
            ROS_INFO("[MappingNode] Still waiting for depth image... (%d/30s)", wait_count);
        }
    }

    if (!g_grid_map->hasDepthObservation()) {
        ROS_WARN("[MappingNode] Depth image not received");
        ROS_WARN("  地图仍会运行，但无障碍物信息");
        ROS_WARN("  请检查:");
        ROS_WARN("    1. RealSense 深度图是否启用? (enable_depth=true)");
        ROS_WARN("    2. 话题名称是否正确?");
    } else {
        ROS_INFO("[MappingNode] Depth image received");
    }

     
    ROS_INFO("[MappingNode] Mapping started!");
     
    ROS_INFO("发布的话题:");
    ROS_INFO("  /grid_map/occupancy         - 原始占据地图");
    ROS_INFO("  /grid_map/occupancy_inflate - 膨胀占据地图 (用于避障)");
     

    // 进入主循环
    ros::spin();

    ROS_INFO("[MappingNode] Node terminated");
    return 0;
}
