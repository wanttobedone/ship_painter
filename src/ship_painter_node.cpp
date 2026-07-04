#include <ros/ros.h>
#include "ship_painter/path_planner.h"
#include <pcl/io/ply_io.h>
#include <boost/filesystem.hpp>
#include <ctime>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>  // 使用PositionTarget替代AttitudeTarget
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "ship_painter/Bspline.h"
#include "ship_painter/BsplineLayer.h"

class ShipPainterNode {
public:
    ShipPainterNode();
    void run();

private:
    // ROS接口
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    // 订阅和发布
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber gps_fix_sub_;              // GPS状态订阅
    ros::Subscriber trajectory_setpoint_sub_; //交接标志位
    ros::Publisher spray_vis_pub_;
    ros::Publisher model_vis_pub_;

    ros::Publisher local_pos_pub_;      // 起飞和设定点需要
    ros::Publisher setpoint_raw_pub_;   // 保留以备兼容
    ros::Publisher path_vis_pub_;       // 路径可视化

    ros::Publisher status_pub_;
    ros::Publisher bspline_pub_;  //发布B样条
    
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;
    
    ros::Timer vis_timer_;

    
    // 路径规划器
    PathPlanner planner_;
    std::vector<PathPlanner::PathLayer> path_layers_;

    std::vector<PathPlanner::BSplineLayer> bspline_layers_;  //添加缓存
    bool bsplines_generated_ = false;  //标志位

    ship_painter::BSpline approach_trajectory_;  // 接近轨迹
    bool has_approach_trajectory_ = false;       // 是否生成了接近轨迹

    ros::Publisher pointcloud_vis_pub_;  // 点云可视化发布者   
    ros::Publisher normal_vis_pub_;  // 法向量可视化发布者
    ros::Publisher alphashape_vis_pub_;//as轮廓发布
    ros::Publisher offset_contour_vis_pub_;  // 偏移轮廓可视化发布者
    ros::Publisher bspline_vis_pub_;//B样条可视化

    //轨迹交接标志
    bool received_trajectory_setpoint_;

    // 变换后的轮廓缓存（map系下，用于可视化）
    std::vector<std::pair<double, std::vector<Eigen::Vector3d>>> map_alphashape_contours_;
    std::vector<std::pair<double, std::vector<Eigen::Vector3d>>> map_offset_contours_;
    
    // 状态管理
    struct FlightState {
        mavros_msgs::State mavros_state;
        geometry_msgs::PoseStamped current_pose;
        bool pose_received = false;

        bool path_ready = false;

        // 冻结任务基准帧 mission0
        struct MissionFrame {
            Eigen::Vector3d origin_map = Eigen::Vector3d::Zero();   // o_m: mission0原点在map中的位置
            double yaw0 = 0.0;                                      // ψ₀: 冻结时刻的yaw（水平面）
            Eigen::Matrix3d R_m = Eigen::Matrix3d::Identity();      // Rz(ψ₀)
            Eigen::Vector3d t_m = Eigen::Vector3d::Zero();          // 部署平移量（局部系内）
            bool valid = false;
        } mission0;

        // RTK状态
        int gps_fix_type = -1;        // -1=未收到, 0=fix, 1=SBAS, 2=GBAS(RTK)
        bool gps_fix_received = false;

    } flight_state_;
    
    // 参数配置
    struct Parameters {
        std::string model_path;
        std::string frame_id = "map";
        std::string mavros_ns = "/mavros";//话题名称，虚拟环境加组号iris_0

        // 任务部署参数（统一坐标方案）
        double mission_forward_distance = 3.0;    // D_f: 目标物近侧到mission0原点的前向距离(m)
        double mission_fcu_ground_height = 0.3;   // h_fcu→ground: 起飞前飞控到地面的高度(m)
        double mission_bottom_clearance = 0.0;    // h_clear: 底部离地安全间隙(m)

        // 喷涂参数
        double spray_radius = SPRAY_RADIUS;
        double spray_distance = SPRAY_DISTANCE;
        double layer_height = SPRAY_WIDTH;

        // 飞行参数
        double takeoff_height = 2.0;
        double flight_speed = 5;        // 巡航速度
        double approach_speed = 0.3;      // 接近速度
        double max_roll = 0.26;           // 最大翻滚角 (15度)
        double max_pitch_adjustment = 0.52; // 最大俯仰调整 (30度)
        double transition_speed = 0.3; //B样条速度

        // 控制模式
        bool use_position_control = false; // 是否使用纯位置控制
        bool enable_spray_visualization = true;

        // 姿态控制增益
        double yaw_gain = 0.8;
        double pitch_gain = 0.8;
        double roll_gain = 0.5;

        bool traverse_clockwise = true;  // 遍历方向：true=顺时针，false=逆时针
        bool require_rtk = false;         // 是否要求RTK Fix后才起飞
        double rtk_wait_timeout = 300.0;  // RTK等待超时(秒)

        // 采样点云保存（调试/离线分析/重建对比）
        bool save_sampled_cloud = false;          // 是否在采样完成后落盘
        std::string sampled_cloud_save_path = ""; // 空则自动生成 /tmp/sampled_<model>_<ts>.ply
    } params_;
    
    // 回调函数
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void visualizationTimerCallback(const ros::TimerEvent& event);

    void trajectorySetpointCallback(const mavros_msgs::PositionTarget::ConstPtr& msg);//交接位

    
    // 飞行控制
    bool waitForConnection();
    bool setMode(const std::string& mode);
    bool arm();
    bool disarm();
    bool takeoffToMapZ(double target_z_map);
    bool land();
    
    void publishBSplineTrajectories();//B样条轨迹
    void generateApproachTrajectory();// 飞向第一层第一个B样条点

    
    // 可视化
    void publishModelVisualization();
    void publishStatusVisualization(); 
    void publishNormalVisualization();  // 法向量可视化
    void publishPointCloudVisualization();//点云可视化
    void publishAlphaShapeVisualization();//as后的路径
    void publishOffsetContourVisualization();//偏移后的路径
    Eigen::Vector3d computeContourCenter(const std::vector<Eigen::Vector3d>& contour);
    void publishBSplineVisualization();//B样条
    
    // 辅助函数
    bool loadParameters();
    bool generateSprayPath();
    void printMissionSummary();
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    double calculateOrientationError(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2);
    Eigen::Vector3d smoothPosition(const Eigen::Vector3d& target, const Eigen::Vector3d& current, double smoothing_factor);
    Eigen::Quaterniond smoothOrientation(const Eigen::Quaterniond& target,const Eigen::Quaterniond& current,double smoothing_factor);
    // 统一坐标部署
    void computeDeploymentTransform(const Eigen::Vector3d& bbox_min,
                                    const Eigen::Vector3d& bbox_max,
                                    Eigen::Vector3d& t_m_out);
    void applyMissionTransform(const Eigen::Matrix3d& R_m,
                               const Eigen::Vector3d& o_m,
                               const Eigen::Vector3d& t_m);
    void reorderLayersByProximity();
};

ShipPainterNode::ShipPainterNode() : nh_private_("~"), received_trajectory_setpoint_(false){
    if (!loadParameters()) {
        ROS_ERROR("Failed to load parameters");
        ros::shutdown();
        return;
    }
    
    
    // 设置订阅者
    state_sub_ = nh_.subscribe(params_.mavros_ns + "/state", 10, 
                               &ShipPainterNode::stateCallback, this);
    pose_sub_ = nh_.subscribe(params_.mavros_ns + "/local_position/pose", 10,
                             &ShipPainterNode::poseCallback, this);
    gps_fix_sub_ = nh_.subscribe(params_.mavros_ns + "/global_position/raw/fix", 10,
                             &ShipPainterNode::gpsFixCallback, this);

    // 设置发布者
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        params_.mavros_ns + "/setpoint_position/local", 10);
    setpoint_raw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
        params_.mavros_ns + "/setpoint_raw/local", 10);
    
    path_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/visualization/path", 10);
    spray_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/visualization/spray", 10);
    model_vis_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/visualization/model", 10);
    status_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/visualization/status", 10);
    // 初始化B样条发布者
    bspline_pub_ = nh_.advertise<ship_painter::BsplineLayer>(
        "/planning/bspline_layers", 10);
    //点云可视化发布
    pointcloud_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
    "/visualization/pointcloud", 10);
    //法向量发布者
    normal_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/visualization/normals", 10);
    alphashape_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
    "/visualization/alphashape_contour", 10);//as轮廓发布

    offset_contour_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
    "/visualization/offset_contour", 10);
    ROS_INFO("Debug visualization publishers initialized");//偏移后轮廓发布

    bspline_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
    "/visualization/bspline", 10);//B样条可视化

    //初始化可视化定时器
    vis_timer_ = nh_.createTimer(
        ros::Duration(0.1),  // 10Hz
        &ShipPainterNode::visualizationTimerCallback, this);
    
    // 设置服务客户端
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
        params_.mavros_ns + "/set_mode");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
        params_.mavros_ns + "/cmd/arming");
    
    // 设置定时器
    
    ROS_INFO("Ship Painter Node initialized");

    //订阅trajectory_server发布的setpoint
    trajectory_setpoint_sub_ = nh_.subscribe(
        params_.mavros_ns + "/setpoint_raw/local", 
        10, 
        &ShipPainterNode::trajectorySetpointCallback, 
        this);
}

bool ShipPainterNode::loadParameters() {
    nh_private_.param<std::string>("model_path", params_.model_path, "");
    nh_private_.param<std::string>("frame_id", params_.frame_id, "map");

    // 任务部署参数
    nh_private_.param<double>("mission_forward_distance", params_.mission_forward_distance, 3.0);
    nh_private_.param<double>("mission_fcu_ground_height", params_.mission_fcu_ground_height, 0.3);
    nh_private_.param<double>("mission_bottom_clearance", params_.mission_bottom_clearance, 0.0);

    nh_private_.param<double>("spray_radius", params_.spray_radius, SPRAY_RADIUS);
    nh_private_.param<double>("spray_distance", params_.spray_distance, SPRAY_DISTANCE);
    nh_private_.param<double>("layer_height", params_.layer_height, SPRAY_WIDTH);

    nh_private_.param<double>("takeoff_height", params_.takeoff_height, 2.0);
    nh_private_.param<double>("flight_speed", params_.flight_speed, 5);
    nh_private_.param<double>("approach_speed", params_.approach_speed, 0.3);
    nh_private_.param<double>("transition_speed", params_.transition_speed, 0.3);

    nh_private_.param<bool>("use_position_control", params_.use_position_control, false);
    nh_private_.param<bool>("enable_spray_visualization", params_.enable_spray_visualization, true);

    double alpha_value;
    nh_private_.param<double>("alpha_shape_value", alpha_value, 3.0);

    int sample_count;
    nh_private_.param<int>("target_sample_count", sample_count, 10000);

    nh_private_.param("traverse_clockwise", params_.traverse_clockwise, true);
    nh_private_.param("require_rtk", params_.require_rtk, false);
    nh_private_.param("rtk_wait_timeout", params_.rtk_wait_timeout, 300.0);
    double resample_spacing_param;
    nh_private_.param<double>("resample_spacing", resample_spacing_param, 0.02);

    nh_private_.param<std::string>("mavros_ns", params_.mavros_ns, "/mavros");

    nh_private_.param<bool>("save_sampled_cloud", params_.save_sampled_cloud, false);
    nh_private_.param<std::string>("sampled_cloud_save_path", params_.sampled_cloud_save_path, "");

    if (params_.model_path.empty()) {
        ROS_ERROR("Model path is empty!");
        return false;
    }
    
    // 配置路径规划器
    PathPlanner::PlannerConfig planner_config;
    planner_config.spray_radius = params_.spray_radius;
    planner_config.spray_distance = params_.spray_distance;
    planner_config.layer_height = params_.layer_height;
    planner_config.max_height_variation = MAX_HEIGHT_DIFF;
    planner_config.use_roll_for_translation = true;
    planner_config.adaptive_pitch = true;
    planner_config.alpha_shape_value = alpha_value;
    planner_config.target_sample_count = static_cast<size_t>(sample_count); // 转换为size_t

    planner_config.resample_spacing = resample_spacing_param;

    planner_config.flight_speed = params_.flight_speed;//飞行速度
    planner_config.transition_speed = params_.transition_speed;//层间过渡速度
    
    planner_ = PathPlanner(planner_config);
    
    ROS_INFO("=== Spray Painting Configuration ===");
    ROS_INFO("Model: %s", params_.model_path.c_str());
    ROS_INFO("Mission forward distance: %.2f m", params_.mission_forward_distance);
    ROS_INFO("Mission FCU ground height: %.2f m", params_.mission_fcu_ground_height);
    ROS_INFO("Mission bottom clearance: %.2f m", params_.mission_bottom_clearance);
    ROS_INFO("Spray radius: %.3f m", params_.spray_radius);
    ROS_INFO("Spray distance: %.3f m", params_.spray_distance);
    ROS_INFO("Layer height: %.3f m", params_.layer_height);
    ROS_INFO("Max height diff per layer: %.3f m", MAX_HEIGHT_DIFF);
    ROS_INFO("Flight speed parameter: %.2f m/s", params_.flight_speed);
    
    return true;
}

bool ShipPainterNode::generateSprayPath() {
    ROS_INFO("Generating spray painting path...");

    // 1. 加载模型
    bool load_success = false;
    std::string ext = params_.model_path.substr(params_.model_path.find_last_of('.'));
    if (ext == ".ply" || ext == ".PLY") {
        load_success = planner_.loadPLYModel(params_.model_path);
    } else {
        load_success = planner_.loadSTLModel(params_.model_path);
    }
    if (!load_success) {
        ROS_ERROR("Failed to load model: %s", params_.model_path.c_str());
        return false;
    }

    // 1.5 可选: 把采样后的原始点云保存到磁盘 (供离线调试 / 重建对比)
    if (params_.save_sampled_cloud) {
        auto cloud = planner_.getOriginalCloud();
        if (!cloud || cloud->empty()) {
            ROS_WARN("save_sampled_cloud=true 但 original_cloud 为空, 跳过保存");
        } else {
            namespace fs = boost::filesystem;
            // 准备时间戳和模型 basename, 用于自动文件名
            auto t = std::time(nullptr);
            char ts[32];
            std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&t));
            std::string basename = fs::path(params_.model_path).stem().string();

            fs::path save_path(params_.sampled_cloud_save_path);
            // 空字符串: 默认 /tmp
            if (save_path.empty()) save_path = "/tmp";
            // 既不是已存在目录, 也不以 .ply 结尾 -> 当成 "目录" 看待 (允许用户写不存在的目录)
            bool treat_as_dir = fs::is_directory(save_path) ||
                                save_path.extension().string() != ".ply";
            if (treat_as_dir) {
                fs::create_directories(save_path);
                save_path /= ("sampled_" + basename + "_" + ts + ".ply");
            } else {
                // 用户给的是文件名, 确保父目录存在
                if (save_path.has_parent_path()) {
                    fs::create_directories(save_path.parent_path());
                }
            }

            if (pcl::io::savePLYFileBinary(save_path.string(), *cloud) == 0) {
                ROS_INFO("Saved sampled cloud (%zu pts) -> %s",
                         cloud->size(), save_path.string().c_str());
            } else {
                ROS_ERROR("Failed to save sampled cloud to %s",
                          save_path.string().c_str());
            }
        }
    }

    // 2. 纯局部系规划（不做任何部署变换）
    path_layers_ = planner_.generateSprayPath();

    if (path_layers_.empty()) {
        ROS_ERROR("No path layers generated");
        return false;
    }

    // 3. 获取局部系包围盒
    Eigen::Vector3d bbox_min, bbox_max;
    planner_.getModelBoundsLocal(bbox_min, bbox_max);

    // 4. 确认 mission0 已冻结
    if (!flight_state_.mission0.valid) {
        ROS_ERROR("mission0 not frozen yet, cannot deploy path");
        return false;
    }

    // 5. 计算部署平移
    Eigen::Vector3d t_m;
    computeDeploymentTransform(bbox_min, bbox_max, t_m);
    flight_state_.mission0.t_m = t_m;  // 保存供可视化使用

    // 6. 统一变换（位置 + 法向 + 姿态，一次性完成）
    applyMissionTransform(flight_state_.mission0.R_m,
                          flight_state_.mission0.origin_map,
                          t_m);

    // 7. 按接近度排序层
    reorderLayersByProximity();

    flight_state_.path_ready = true;
    ROS_INFO("Path generation and deployment complete");

    return true;
}

void ShipPainterNode::printMissionSummary() {
    ROS_INFO("=== Mission Summary ===");
    ROS_INFO("Total layers: %zu", path_layers_.size());
    
    size_t total_waypoints = 0;
    for (size_t i = 0; i < path_layers_.size(); ++i) {
        const auto& layer = path_layers_[i];
        size_t layer_waypoints = layer.waypoints.size();
        total_waypoints += layer_waypoints;
        ROS_INFO("  Layer %zu: z=%.3f m, %zu waypoints", 
                 i+1, layer.z_center, layer_waypoints);
    }
    
    ROS_INFO("Total waypoints: %zu", total_waypoints);
    
}

void ShipPainterNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    flight_state_.mavros_state = *msg;
}

void ShipPainterNode::gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    flight_state_.gps_fix_type = msg->status.status;
    flight_state_.gps_fix_received = true;
}

void ShipPainterNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    flight_state_.current_pose = *msg;

    if (!flight_state_.pose_received) {
        flight_state_.pose_received = true;
        ROS_INFO("First pose received");
    }
}

double ShipPainterNode::calculateDistance(const geometry_msgs::Point& p1,
                                         const geometry_msgs::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double ShipPainterNode::calculateOrientationError(const geometry_msgs::Quaternion& q1,
                                                 const geometry_msgs::Quaternion& q2) {
    Eigen::Quaterniond eigen_q1(q1.w, q1.x, q1.y, q1.z);
    Eigen::Quaterniond eigen_q2(q2.w, q2.x, q2.y, q2.z);
    return eigen_q1.angularDistance(eigen_q2);
}

Eigen::Vector3d ShipPainterNode::smoothPosition(const Eigen::Vector3d& target,
                                               const Eigen::Vector3d& current,
                                               double smoothing_factor) {
    return current + smoothing_factor * (target - current);
}

Eigen::Quaterniond ShipPainterNode::smoothOrientation(const Eigen::Quaterniond& target,
                                                     const Eigen::Quaterniond& current,
                                                     double smoothing_factor) {
    return current.slerp(smoothing_factor, target);
}

// 可视化函数
void ShipPainterNode::visualizationTimerCallback(const ros::TimerEvent& event) {
    publishModelVisualization();
    publishStatusVisualization();
    publishPointCloudVisualization();//点云可视化
    publishNormalVisualization();//法向量可视化
    publishAlphaShapeVisualization(); //as轮廓可视化
    publishOffsetContourVisualization();//偏移轮廓可视化
    publishBSplineVisualization();  //B样条曲线（青色线）
}

void ShipPainterNode::publishAlphaShapeVisualization() {
    if (map_alphashape_contours_.empty()) return;

    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < map_alphashape_contours_.size(); ++i) {
        const auto& contour = map_alphashape_contours_[i].second;
        if (contour.size() < 2) continue;

        visualization_msgs::Marker contour_marker;
        contour_marker.header.frame_id = "map";
        contour_marker.header.stamp = ros::Time::now();
        contour_marker.ns = "alphashape_contour";
        contour_marker.id = i;
        contour_marker.type = visualization_msgs::Marker::LINE_STRIP;
        contour_marker.action = visualization_msgs::Marker::ADD;
        contour_marker.scale.x = 0.03;

        contour_marker.color.r = 1.0;
        contour_marker.color.g = 0.0;
        contour_marker.color.b = 0.0;
        contour_marker.color.a = 0.8;

        for (const auto& pt : contour) {
            geometry_msgs::Point p;
            p.x = pt.x(); p.y = pt.y(); p.z = pt.z();
            contour_marker.points.push_back(p);
        }

        // 闭合
        if (contour.size() > 2) {
            geometry_msgs::Point p;
            p.x = contour[0].x(); p.y = contour[0].y(); p.z = contour[0].z();
            contour_marker.points.push_back(p);
        }

        marker_array.markers.push_back(contour_marker);
    }

    alphashape_vis_pub_.publish(marker_array);
}

void ShipPainterNode::publishOffsetContourVisualization() {
    if (map_offset_contours_.empty()) return;

    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < map_offset_contours_.size(); ++i) {
        double z_center = map_offset_contours_[i].first;
        const auto& contour = map_offset_contours_[i].second;
        if (contour.size() < 2) continue;

        // 偏移轮廓线条（蓝色）
        visualization_msgs::Marker contour_marker;
        contour_marker.header.frame_id = "map";
        contour_marker.header.stamp = ros::Time::now();
        contour_marker.ns = "offset_contour";
        contour_marker.id = i;
        contour_marker.type = visualization_msgs::Marker::LINE_STRIP;
        contour_marker.action = visualization_msgs::Marker::ADD;
        contour_marker.scale.x = 0.04;
        contour_marker.color.r = 0.0;
        contour_marker.color.g = 0.4;
        contour_marker.color.b = 1.0;
        contour_marker.color.a = 0.9;

        for (const auto& pt : contour) {
            geometry_msgs::Point p;
            p.x = pt.x(); p.y = pt.y(); p.z = pt.z() + 0.01;
            contour_marker.points.push_back(p);
        }

        if (contour.size() > 2) {
            geometry_msgs::Point p;
            p.x = contour[0].x(); p.y = contour[0].y(); p.z = contour[0].z() + 0.01;
            contour_marker.points.push_back(p);
        }

        marker_array.markers.push_back(contour_marker);

        // 蓝色小球
        visualization_msgs::Marker points_marker;
        points_marker.header = contour_marker.header;
        points_marker.ns = "offset_contour_points";
        points_marker.id = i + 1000;
        points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        points_marker.action = visualization_msgs::Marker::ADD;
        points_marker.scale.x = 0.02;
        points_marker.scale.y = 0.02;
        points_marker.scale.z = 0.02;
        points_marker.color.r = 0.0;
        points_marker.color.g = 0.2;
        points_marker.color.b = 0.8;
        points_marker.color.a = 0.8;

        size_t step = std::max(1UL, contour.size() / 50);
        for (size_t j = 0; j < contour.size(); j += step) {
            geometry_msgs::Point p;
            p.x = contour[j].x(); p.y = contour[j].y(); p.z = contour[j].z() + 0.02;
            points_marker.points.push_back(p);
        }
        marker_array.markers.push_back(points_marker);

        // 层标签
        visualization_msgs::Marker text_marker;
        text_marker.header = contour_marker.header;
        text_marker.ns = "offset_contour_labels";
        text_marker.id = i + 2000;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;

        Eigen::Vector3d center = computeContourCenter(contour);
        text_marker.pose.position.x = center.x();
        text_marker.pose.position.y = center.y();
        text_marker.pose.position.z = center.z() + 0.1;
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.1;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;

        std::ostringstream ss;
        ss << "Layer " << (i+1) << "\nz=" << std::fixed << std::setprecision(2) << z_center
           << "\n" << contour.size() << " pts";
        text_marker.text = ss.str();
        marker_array.markers.push_back(text_marker);
    }

    offset_contour_vis_pub_.publish(marker_array);
}
// 辅助函数：计算轮廓中心（如果不存在的话）
Eigen::Vector3d ShipPainterNode::computeContourCenter(const std::vector<Eigen::Vector3d>& contour) {
    if (contour.empty()) return Eigen::Vector3d::Zero();
    
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto& pt : contour) {
        center += pt;
    }
    center /= contour.size();
    return center;
}

void ShipPainterNode::publishModelVisualization() {
    // PLY文件不支持MESH_RESOURCE，使用点云可视化代替
    std::string ext = params_.model_path.substr(params_.model_path.find_last_of('.'));
    if (ext == ".ply" || ext == ".PLY") {
        return;  // PLY模型通过publishPointCloudVisualization()显示
    }

    if (!flight_state_.mission0.valid) return;

    // 用 mission transform 计算模型在 map 系中的位置
    // STL 模型原点在局部系原点，需要施加 t_m + R_m 变换
    Eigen::Vector3d model_pos_map = flight_state_.mission0.origin_map +
        flight_state_.mission0.R_m * flight_state_.mission0.t_m;

    visualization_msgs::Marker model_marker;
    model_marker.header.frame_id = "map";
    model_marker.header.stamp = ros::Time::now();
    model_marker.ns = "ship_model";
    model_marker.id = 0;
    model_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    model_marker.mesh_resource = "file://" + params_.model_path;
    model_marker.action = visualization_msgs::Marker::ADD;

    model_marker.pose.position.x = model_pos_map.x();
    model_marker.pose.position.y = model_pos_map.y();
    model_marker.pose.position.z = model_pos_map.z();

    // 朝向由 R_m（仅 yaw 旋转）给出
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, flight_state_.mission0.yaw0);
    model_marker.pose.orientation.x = q.x();
    model_marker.pose.orientation.y = q.y();
    model_marker.pose.orientation.z = q.z();
    model_marker.pose.orientation.w = q.w();

    model_marker.scale.x = 1.0;
    model_marker.scale.y = 1.0;
    model_marker.scale.z = 1.0;

    model_marker.color.r = 0.7;
    model_marker.color.g = 0.7;
    model_marker.color.b = 0.7;
    model_marker.color.a = 0.8;

    model_vis_pub_.publish(model_marker);
}

void ShipPainterNode::publishStatusVisualization() {
    visualization_msgs::Marker status_marker;
    status_marker.header.frame_id = "map";
    status_marker.header.stamp = ros::Time::now();
    status_marker.ns = "status";
    status_marker.id = 0;
    status_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    status_marker.action = visualization_msgs::Marker::ADD;
    
    // 状态文字显示在 mission0 前方上方
    if (flight_state_.mission0.valid) {
        Eigen::Vector3d status_pos = flight_state_.mission0.origin_map +
            flight_state_.mission0.R_m * Eigen::Vector3d(params_.mission_forward_distance, 0, 2.0);
        status_marker.pose.position.x = status_pos.x();
        status_marker.pose.position.y = status_pos.y();
        status_marker.pose.position.z = status_pos.z();
    } else {
        status_marker.pose.position.x = 0;
        status_marker.pose.position.y = 0;
        status_marker.pose.position.z = 2.0;
    }
    status_marker.pose.orientation.w = 1.0;
    
    status_marker.scale.z = 0.3;
    status_marker.color.r = 1.0;
    status_marker.color.g = 1.0;
    status_marker.color.b = 1.0;
    status_marker.color.a = 1.0;
    
    std::stringstream ss;
    if (flight_state_.path_ready) {
        ss << "B-Spline Trajectory Active\n";
        ss << "Total layers: " << path_layers_.size() << "\n";
        ss << "Mode: Continuous tracking";
    } else {
        ss << "Initializing...";
    }
    
    status_marker.text = ss.str();
    status_pub_.publish(status_marker);
}

// 4. 实现点云可视化函数
void ShipPainterNode::publishPointCloudVisualization() {

    
    visualization_msgs::MarkerArray marker_array;
    
    // 获取处理后的点云
    pcl::PointCloud<pcl::PointNormal>::Ptr processed_cloud = planner_.getProcessedCloud();
    if (!processed_cloud || processed_cloud->empty()) {
        return;
    }
    
    // 创建处理后点云的可视化标记
    visualization_msgs::Marker processed_marker;
    processed_marker.header.frame_id = params_.frame_id;
    processed_marker.header.stamp = ros::Time::now();
    processed_marker.ns = "processed_pointcloud";
    processed_marker.id = 0;
    processed_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    processed_marker.action = visualization_msgs::Marker::ADD;
    
    // 设置点的大小
    processed_marker.scale.x = 0.1;  // 1.5cm直径的小球
    processed_marker.scale.y = 0.1;
    processed_marker.scale.z = 0.1;
    
    // 设置颜色为白色
    processed_marker.color.r = 1.0;
    processed_marker.color.g = 1.0;
    processed_marker.color.b = 1.0;
    processed_marker.color.a = 0.8;   ///透明度
    
    // 添加所有点（应用 mission transform）
    for (const auto& point : processed_cloud->points) {
        if (!pcl::isFinite(point)) continue;

        Eigen::Vector3d pt_local(point.x, point.y, point.z);
        Eigen::Vector3d pt_map = flight_state_.mission0.valid
            ? flight_state_.mission0.origin_map + flight_state_.mission0.R_m * (pt_local + flight_state_.mission0.t_m)
            : pt_local;

        geometry_msgs::Point p;
        p.x = pt_map.x(); p.y = pt_map.y(); p.z = pt_map.z();
        processed_marker.points.push_back(p);
    }
    
    marker_array.markers.push_back(processed_marker);

    //原始点云可视化
    pcl::PointCloud<pcl::PointNormal>::Ptr original_cloud = planner_.getOriginalCloud();
    if (original_cloud && !original_cloud->empty()) {
        visualization_msgs::Marker original_marker;
        original_marker.header = processed_marker.header;
        original_marker.ns = "original_pointcloud";
        original_marker.id = 1;
        original_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        original_marker.action = visualization_msgs::Marker::ADD;
        
        // 原始点云稍微小一点
        original_marker.scale.x = 0.01;
        original_marker.scale.y = 0.01;
        original_marker.scale.z = 0.01;
        
        // 设置颜色为淡灰色
        original_marker.color.r = 0.1;
        original_marker.color.g = 0.0;
        original_marker.color.b = 0.0;
        original_marker.color.a = 0.8;   // 透明度
        
        // 添加点，但只取部分点以减少计算负担
        size_t step = std::max(1UL, original_cloud->size() / 10000);  // 最多显示10000个点
        for (size_t i = 0; i < original_cloud->size(); i += step) {
            const auto& point = original_cloud->points[i];
            if (!pcl::isFinite(point)) {
                continue;
            }
            
            Eigen::Vector3d pt_local(point.x, point.y, point.z);
            Eigen::Vector3d pt_map = flight_state_.mission0.valid
                ? flight_state_.mission0.origin_map + flight_state_.mission0.R_m * (pt_local + flight_state_.mission0.t_m)
                : pt_local;

            geometry_msgs::Point p;
            p.x = pt_map.x(); p.y = pt_map.y(); p.z = pt_map.z();
            original_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(original_marker);
    }

    
        // 添加点云中心可视化
    if (processed_cloud && !processed_cloud->empty()) {
        // 计算点云中心
        Eigen::Vector3d center(0, 0, 0);
        for (const auto& pt : processed_cloud->points) {
            center += Eigen::Vector3d(pt.x, pt.y, pt.z);
        }
        center /= processed_cloud->size();
        
        // 应用 mission transform
        if (flight_state_.mission0.valid) {
            center = flight_state_.mission0.origin_map +
                flight_state_.mission0.R_m * (center + flight_state_.mission0.t_m);
        }
        
        // 创建中心点标记
        visualization_msgs::Marker center_marker;
        center_marker.header.frame_id = params_.frame_id;
        center_marker.header.stamp = ros::Time::now();
        center_marker.ns = "pointcloud_center";
        center_marker.id = 0;
        center_marker.type = visualization_msgs::Marker::SPHERE;
        center_marker.action = visualization_msgs::Marker::ADD;
        
        center_marker.pose.position.x = center.x();
        center_marker.pose.position.y = center.y();
        center_marker.pose.position.z = center.z();
        center_marker.pose.orientation.w = 1.0;
        
        center_marker.scale.x = 0.1;  // 10cm直径
        center_marker.scale.y = 0.1;
        center_marker.scale.z = 0.1;
        
        center_marker.color.r = 0.0;
        center_marker.color.g = 1.0;  // 绿色
        center_marker.color.b = 0.0;
        center_marker.color.a = 1.0;
        
        marker_array.markers.push_back(center_marker);
    }
    
    // 发布标记数组
    pointcloud_vis_pub_.publish(marker_array);
    
    // 打印调试信息（只在第一次或偶尔打印）
    // static int debug_counter = 0;
    // if (debug_counter % 50 == 0) {  // 每5秒打印一次（假设10Hz调用频率）
    //     ROS_INFO_THROTTLE(5, "Publishing pointcloud visualization: %zu processed points,%zu original points",
    //                      processed_cloud->size(), original_cloud ? original_cloud->size() : 0);
    // }
    // debug_counter++;
}

//法向量可视化函数
void ShipPainterNode::publishNormalVisualization() {
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;

    // 辅助 lambda：判断是否近似纯X或纯Z向
    auto is_near_axis_x_or_z = [](const Eigen::Vector3d& v)->bool {
        Eigen::Vector3d n = v.normalized();
        // 纯X (x ~ ±1, y,z ~ 0) 或 纯Z (z ~ ±1, x,y ~ 0)
        bool pureX = (std::fabs(n.x()) > 0.99 && std::fabs(n.y()) < 0.1 && std::fabs(n.z()) < 0.1);
        bool pureZ = (std::fabs(n.z()) > 0.99 && std::fabs(n.x()) < 0.1 && std::fabs(n.y()) < 0.1);
        return pureX || pureZ;
    };

    // 1. 点云法向量（绿色箭头，采样）
    pcl::PointCloud<pcl::PointNormal>::Ptr processed_cloud = planner_.getProcessedCloud();
    if (processed_cloud && !processed_cloud->empty()) {
        size_t step = std::max(1UL, processed_cloud->size() / 1000); // 最多显示1000个
        double normal_length = 0.05; // 5cm

        for (size_t i = 0; i < processed_cloud->size(); i += step) {
            const auto& point = processed_cloud->points[i];
            if (!pcl::isFinite(point)) continue;

            Eigen::Vector3d n(point.normal_x, point.normal_y, point.normal_z);
            if (n.norm() < 1e-6) continue;
            if (std::fabs(point.normal_x) < 1e-3 && std::fabs(point.normal_y) < 1e-3) continue; // 过滤/纯Z向

            visualization_msgs::Marker arrow;
            arrow.header.frame_id = params_.frame_id;
            arrow.header.stamp = ros::Time::now();
            arrow.ns = "pointcloud_normals";
            arrow.id = marker_id++;
            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.action = visualization_msgs::Marker::ADD;
            arrow.scale.x = 0.005; // 箭杆直径
            arrow.scale.y = 0.01;  // 箭头宽
            arrow.scale.z = 0.01;  // 箭头长
            arrow.color.r = 0.0;
            arrow.color.g = 1.0;
            arrow.color.b = 0.0;
            arrow.color.a = 0.6;

            Eigen::Vector3d pt_local(point.x, point.y, point.z);
            Eigen::Vector3d pt_map = flight_state_.mission0.valid
                ? flight_state_.mission0.origin_map + flight_state_.mission0.R_m * (pt_local + flight_state_.mission0.t_m)
                : pt_local;
            Eigen::Vector3d n_map = flight_state_.mission0.valid
                ? flight_state_.mission0.R_m * n.normalized()
                : n.normalized();

            geometry_msgs::Point start, end;
            start.x = pt_map.x(); start.y = pt_map.y(); start.z = pt_map.z();

            Eigen::Vector3d end_e = pt_map + n_map * normal_length;
            end.x = end_e.x(); end.y = end_e.y(); end.z = end_e.z();

            arrow.points.clear();
            arrow.points.push_back(start);
            arrow.points.push_back(end);

            marker_array.markers.push_back(arrow);
        }
    }

    // 2. 路径点法向量（蓝色箭头，过滤纯X/纯Z）
    if (flight_state_.path_ready && !path_layers_.empty()) {
        double wp_normal_length = 0.08; // 8cm

        for (const auto& layer : path_layers_) {
            for (const auto& wp : layer.waypoints) {
                Eigen::Vector3d n = wp.surface_normal;
                if (n.norm() < 1e-6) continue;
                // if (is_near_axis_x_or_z(n)) continue; // 过滤

                visualization_msgs::Marker arrow;
                arrow.header.frame_id = params_.frame_id;
                arrow.header.stamp = ros::Time::now();
                arrow.ns = "waypoint_normals";
                arrow.id = marker_id++;
                arrow.type = visualization_msgs::Marker::ARROW;
                arrow.action = visualization_msgs::Marker::ADD;
                arrow.scale.x = 0.008;
                arrow.scale.y = 0.02;
                arrow.scale.z = 0.02;
                arrow.color.r = 0.0;
                arrow.color.g = 0.0;
                arrow.color.b = 1.0;
                arrow.color.a = 0.8;

                geometry_msgs::Point start, end;
                start.x = wp.position.x();
                start.y = wp.position.y();
                start.z = wp.position.z();

                Eigen::Vector3d end_e = Eigen::Vector3d(start.x, start.y, start.z) + n.normalized() * wp_normal_length;
                end.x = end_e.x(); end.y = end_e.y(); end.z = end_e.z();

                arrow.points.clear();
                arrow.points.push_back(start);
                arrow.points.push_back(end);

                marker_array.markers.push_back(arrow);
            }
        }
    }

    normal_vis_pub_.publish(marker_array);

    // static int debug_counter = 0;
    // if (debug_counter % 100 == 0) {
    //     ROS_INFO_THROTTLE(10, "Publishing normal vectors visualization (arrows): %zu markers", marker_array.markers.size());
    // }
    // debug_counter++;
}

//B样条轨迹
void ShipPainterNode::publishBSplineTrajectories() {
    // 生成B样条轨迹
    if (!bsplines_generated_) {
        bspline_layers_ = planner_.generateBSplineLayersFromPath(path_layers_);
        bsplines_generated_ = true;
        ROS_INFO("B-spline layers generated from transformed path (drone-relative)");
    }
}

// 发布B样条可视化
void ShipPainterNode::publishBSplineVisualization() {
    if (!bsplines_generated_) return;
    
    visualization_msgs::MarkerArray markers;
    
    for (size_t i = 0; i < bspline_layers_.size(); i++) {
        const auto& layer = bspline_layers_[i];
        
        // B样条曲线
        visualization_msgs::Marker spline_marker;
        spline_marker.header.frame_id = "map";
        spline_marker.header.stamp = ros::Time::now();
        spline_marker.ns = "bspline_curves";
        spline_marker.id = i;
        spline_marker.type = visualization_msgs::Marker::LINE_STRIP;
        spline_marker.action = visualization_msgs::Marker::ADD;
        spline_marker.scale.x = 0.03;  // 比航点线粗一点
        spline_marker.color.r = 0.0;
        spline_marker.color.g = 1.0;
        spline_marker.color.b = 1.0;  // 青色
        spline_marker.color.a = 0.8;
        
        // 采样B样条曲线
        double total_time = layer.trajectory.getTotalTime();
        for (double t = 0; t <= total_time; t += 0.05) {  // 每0.05秒一个点
            Eigen::Vector3d pos = layer.trajectory.getPosition(t);
            geometry_msgs::Point p;
            p.x = pos.x();
            p.y = pos.y();
            p.z = pos.z();
            spline_marker.points.push_back(p);
        }
        
        markers.markers.push_back(spline_marker);
        
        // 控制点可视化（用小球表示）
        visualization_msgs::Marker control_pts_marker;
        control_pts_marker.header = spline_marker.header;
        control_pts_marker.ns = "bspline_control_points";
        control_pts_marker.id = i + 1000;
        control_pts_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        control_pts_marker.action = visualization_msgs::Marker::ADD;
        control_pts_marker.scale.x = 0.08;
        control_pts_marker.scale.y = 0.08;
        control_pts_marker.scale.z = 0.08;
        control_pts_marker.color.r = 1.0;
        control_pts_marker.color.g = 0.0;
        control_pts_marker.color.b = 1.0;  // 紫色
        control_pts_marker.color.a = 0.6;
        
        for (const auto& cp : layer.trajectory.getControlPoints()) {
            geometry_msgs::Point p;
            p.x = cp.x();
            p.y = cp.y();
            p.z = cp.z();
            control_pts_marker.points.push_back(p);
        }
        
        markers.markers.push_back(control_pts_marker);
    }
    
    bspline_vis_pub_.publish(markers);
}

// 主要任务执行函数保持不变...
bool ShipPainterNode::waitForConnection() {
    ROS_INFO("Waiting for MAVROS connection...");
    ros::Rate rate(20);
    for (int i = 0; i < 1000; ++i) {
        if (flight_state_.mavros_state.connected) {
            ROS_INFO("MAVROS connected!");
            return true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_ERROR("Timeout waiting for MAVROS connection");
    return false;
}

bool ShipPainterNode::setMode(const std::string& mode) {
    mavros_msgs::SetMode srv;
    srv.request.custom_mode = mode;
    
    if (set_mode_client_.call(srv) && srv.response.mode_sent) {
        ROS_INFO("Mode set to %s", mode.c_str());
        return true;
    }
    
    ROS_WARN("Failed to set mode to %s", mode.c_str());
    return false;
}

bool ShipPainterNode::arm() {
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    
    if (arming_client_.call(srv) && srv.response.success) {
        ROS_INFO("Vehicle armed");
        return true;
    }
    
    ROS_WARN("Failed to arm vehicle");
    return false;
}

bool ShipPainterNode::disarm() {
    mavros_msgs::CommandBool srv;
    srv.request.value = false;
    
    if (arming_client_.call(srv) && srv.response.success) {
        ROS_INFO("Vehicle disarmed");
        return true;
    }
    
    return false;
}

bool ShipPainterNode::takeoffToMapZ(double target_z_map) {
    ROS_INFO("Taking off to map z = %.2f m...", target_z_map);

    geometry_msgs::PoseStamped takeoff_pose = flight_state_.current_pose;
    takeoff_pose.header.frame_id = "map";
    takeoff_pose.pose.position.x = flight_state_.mission0.origin_map.x();
    takeoff_pose.pose.position.y = flight_state_.mission0.origin_map.y();
    takeoff_pose.pose.position.z = target_z_map;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, flight_state_.mission0.yaw0);
    takeoff_pose.pose.orientation.x = q.x();
    takeoff_pose.pose.orientation.y = q.y();
    takeoff_pose.pose.orientation.z = q.z();
    takeoff_pose.pose.orientation.w = q.w();

    ros::Rate rate(20);
    while (ros::ok()) {
        takeoff_pose.header.stamp = ros::Time::now();
        local_pos_pub_.publish(takeoff_pose);

        double current_z = flight_state_.current_pose.pose.position.z;

        if (std::abs(current_z - target_z_map) < 0.2) {
            ROS_INFO("Takeoff complete at map z = %.2f m", current_z);
            return true;
        }

        ROS_INFO_THROTTLE(
            1,
            "Taking off... Current map z: %.2f, Target map z: %.2f, Relative target: %.2f",
            current_z,
            target_z_map,
            target_z_map - flight_state_.mission0.origin_map.z()
        );

        ros::spinOnce();
        rate.sleep();
    }

    return false;
}

void ShipPainterNode::run() {
    // 1. 等待连接
    if (!waitForConnection()) {
        return;
    }
    
    // 等待位置数据和EKF2初始化
    // 等待 local_position/pose 可用
    // 注意：实机中 PX4 local origin 不一定等于起飞点，
    // 所以不能用 fabs(z)<2.0 判断 EKF/local pose 是否 ready。
    ROS_INFO("Waiting for local position data and basic pose validity...");
    ros::Rate rate(20);
    ros::Time ekf_wait_start = ros::Time::now();
    bool ekf_ready = false;

    while (ros::ok() && !ekf_ready) {
        ros::spinOnce();

        if (flight_state_.pose_received) {
            const auto& p = flight_state_.current_pose.pose.position;
            const auto& q = flight_state_.current_pose.pose.orientation;

            const double q_norm = std::sqrt(
                q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
            );

            const bool pose_finite =
                std::isfinite(p.x) &&
                std::isfinite(p.y) &&
                std::isfinite(p.z) &&
                std::isfinite(q.x) &&
                std::isfinite(q.y) &&
                std::isfinite(q.z) &&
                std::isfinite(q.w);

            const bool quat_ok =
                std::isfinite(q_norm) &&
                q_norm > 0.7 &&
                q_norm < 1.3;

            if (pose_finite && quat_ok) {
                ekf_ready = true;

                tf2::Quaternion cur_q(q.x, q.y, q.z, q.w);
                double roll, pitch, yaw;
                tf2::Matrix3x3(cur_q).getRPY(roll, pitch, yaw);

                ROS_INFO("Local pose ready: position=(%.2f, %.2f, %.2f), yaw=%.1f deg. "
                        "mission0 stability will be checked next.",
                        p.x,
                        p.y,
                        p.z,
                        yaw * 180.0 / M_PI);
            } else {
                ROS_WARN_THROTTLE(
                    5.0,
                    "Waiting for valid local pose: position=(%.2f, %.2f, %.2f), q_norm=%.3f",
                    p.x,
                    p.y,
                    p.z,
                    q_norm
                );
            }
        } else {
            ROS_WARN_THROTTLE(5.0, "Waiting for /mavros/local_position/pose...");
        }

        if ((ros::Time::now() - ekf_wait_start).toSec() > 30.0) {
            if (flight_state_.pose_received) {
                ROS_ERROR("Local pose wait timeout! Current position=(%.2f, %.2f, %.2f)",
                        flight_state_.current_pose.pose.position.x,
                        flight_state_.current_pose.pose.position.y,
                        flight_state_.current_pose.pose.position.z);
            } else {
                ROS_ERROR("Local pose wait timeout: no /mavros/local_position/pose received");
            }

            ROS_ERROR("Please check:");
            ROS_ERROR("  1. Is MAVROS connected to PX4?");
            ROS_ERROR("  2. Is /mavros/local_position/pose being published?");
            ROS_ERROR("  3. Is PX4 EKF producing local position?");
            return;
        }

        rate.sleep();
    }

    // =========================================================================
    // 冻结 mission0 任务基准帧
    // 在EKF稳定后，等待位置和yaw连续稳定再冻结
    // =========================================================================
    ROS_INFO("Freezing mission0 reference frame...");
    {
        const int stable_frames_required = 20;  // 连续稳定帧数（20帧 = 1秒 @20Hz）
        const double pos_jitter_threshold = 0.05;  // 位置跳变阈值(m)
        const double yaw_jitter_threshold = 0.05;  // yaw跳变阈值(rad, ~3°)
        int stable_count = 0;
        Eigen::Vector3d prev_pos(
            flight_state_.current_pose.pose.position.x,
            flight_state_.current_pose.pose.position.y,
            flight_state_.current_pose.pose.position.z);

        tf2::Quaternion prev_q(
            flight_state_.current_pose.pose.orientation.x,
            flight_state_.current_pose.pose.orientation.y,
            flight_state_.current_pose.pose.orientation.z,
            flight_state_.current_pose.pose.orientation.w);
        double prev_roll, prev_pitch, prev_yaw;
        tf2::Matrix3x3(prev_q).getRPY(prev_roll, prev_pitch, prev_yaw);

        ros::Time freeze_wait_start = ros::Time::now();

        while (ros::ok()) {
            ros::spinOnce();

            Eigen::Vector3d cur_pos(
                flight_state_.current_pose.pose.position.x,
                flight_state_.current_pose.pose.position.y,
                flight_state_.current_pose.pose.position.z);

            tf2::Quaternion cur_q(
                flight_state_.current_pose.pose.orientation.x,
                flight_state_.current_pose.pose.orientation.y,
                flight_state_.current_pose.pose.orientation.z,
                flight_state_.current_pose.pose.orientation.w);
            double cur_roll, cur_pitch, cur_yaw;
            tf2::Matrix3x3(cur_q).getRPY(cur_roll, cur_pitch, cur_yaw);

            double pos_diff = (cur_pos - prev_pos).norm();
            double yaw_diff = fabs(cur_yaw - prev_yaw);
            if (yaw_diff > M_PI) yaw_diff = 2.0 * M_PI - yaw_diff;

            if (pos_diff < pos_jitter_threshold && yaw_diff < yaw_jitter_threshold) {
                stable_count++;
            } else {
                stable_count = 0;
            }

            prev_pos = cur_pos;
            prev_yaw = cur_yaw;

            if (stable_count >= stable_frames_required) {
                // 冻结 mission0
                flight_state_.mission0.origin_map = cur_pos;
                flight_state_.mission0.yaw0 = cur_yaw;
                // R_m = Rz(yaw0)，只取yaw，不继承roll/pitch
                double c = cos(cur_yaw), s = sin(cur_yaw);
                flight_state_.mission0.R_m <<
                    c, -s, 0,
                    s,  c, 0,
                    0,  0, 1;
                flight_state_.mission0.valid = true;
                ROS_INFO("mission0 frozen: origin=(%.2f, %.2f, %.2f), yaw=%.1f deg",
                         cur_pos.x(), cur_pos.y(), cur_pos.z(),
                         cur_yaw * 180.0 / M_PI);
                break;
            }

            if ((ros::Time::now() - freeze_wait_start).toSec() > 15.0) {
                // 超时则用当前帧强制冻结
                ROS_WARN("mission0 stability timeout, freezing with current pose");
                flight_state_.mission0.origin_map = cur_pos;
                flight_state_.mission0.yaw0 = cur_yaw;
                double c = cos(cur_yaw), s = sin(cur_yaw);
                flight_state_.mission0.R_m <<
                    c, -s, 0,
                    s,  c, 0,
                    0,  0, 1;
                flight_state_.mission0.valid = true;
                break;
            }

            rate.sleep();
        }
    }

    ROS_INFO("EKF2 ready, mission0 frozen");

    // 2.5 RTK收敛等待
    if (params_.require_rtk) {
        ROS_INFO("=== RTK mode enabled, waiting for RTK Fix... ===");
        ros::Time rtk_wait_start = ros::Time::now();
        bool rtk_fixed = false;

        while (ros::ok() && !rtk_fixed) {
            ros::spinOnce();

            if (flight_state_.gps_fix_received) {
                // NavSatFix status: -1=NO_FIX, 0=FIX, 1=SBAS, 2=GBAS(RTK Fix)
                if (flight_state_.gps_fix_type >= sensor_msgs::NavSatStatus::STATUS_GBAS_FIX) {
                    rtk_fixed = true;
                    ROS_INFO("RTK Fixed! GPS status=%d, proceeding with high-precision position.",
                             flight_state_.gps_fix_type);
                } else {
                    static ros::Time last_rtk_log = ros::Time::now();
                    if ((ros::Time::now() - last_rtk_log).toSec() > 5.0) {
                        const char* status_str = "UNKNOWN";
                        switch (flight_state_.gps_fix_type) {
                            case -1: status_str = "NO_FIX"; break;
                            case 0:  status_str = "GPS_FIX"; break;
                            case 1:  status_str = "SBAS/RTK_FLOAT"; break;
                        }
                        double elapsed = (ros::Time::now() - rtk_wait_start).toSec();
                        ROS_WARN("Waiting for RTK Fix... current=%s, elapsed=%.0fs/%.0fs",
                                 status_str, elapsed, params_.rtk_wait_timeout);
                        last_rtk_log = ros::Time::now();
                    }
                }
            }

            // 超时处理
            if ((ros::Time::now() - rtk_wait_start).toSec() > params_.rtk_wait_timeout) {
                ROS_ERROR("RTK Fix timeout (%.0fs)! Current GPS status=%d",
                          params_.rtk_wait_timeout, flight_state_.gps_fix_type);
                ROS_ERROR("Aborting mission. Check RTK base station and sky visibility.");
                return;
            }

            rate.sleep();
        }
    } else {
        ROS_INFO("RTK not required (require_rtk=false), using current GPS.");
    }

    // 3. 生成喷涂路径
    if (!generateSprayPath()) {
        ROS_ERROR("Failed to generate spray path");
        return;
    }
    
    // 4. 发送初始设定点（基于 mission0 原点）
    ROS_INFO("Sending initial setpoints...");
    geometry_msgs::PoseStamped init_pose;
    init_pose.header.frame_id = "map";
    double takeoff_target_z =
    flight_state_.mission0.origin_map.z() + params_.takeoff_height;

    init_pose.pose.position.x = flight_state_.mission0.origin_map.x();
    init_pose.pose.position.y = flight_state_.mission0.origin_map.y();
    init_pose.pose.position.z = takeoff_target_z;
    // 用 mission0.yaw0 构造朝向四元数
    tf2::Quaternion init_q;
    init_q.setRPY(0.0, 0.0, flight_state_.mission0.yaw0);
    init_pose.pose.orientation.x = init_q.x();
    init_pose.pose.orientation.y = init_q.y();
    init_pose.pose.orientation.z = init_q.z();
    init_pose.pose.orientation.w = init_q.w();
    
    for (int i = 0; i < 40; ++i) {
        init_pose.header.stamp = ros::Time::now();
        local_pos_pub_.publish(init_pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    // 5. 设置OFFBOARD模式
    if (!setMode("OFFBOARD")) {
        ROS_ERROR("Failed to set OFFBOARD mode");
        return;
    }
    
    // 6. 解锁
    bool armed_success = false;
    for (int i = 0; i < 10; ++i) {
        init_pose.header.stamp = ros::Time::now();
        local_pos_pub_.publish(init_pose);
        
        if (arm()) {
            for (int j = 0; j < 20; ++j) {
                init_pose.header.stamp = ros::Time::now();
                local_pos_pub_.publish(init_pose);
                ros::spinOnce();
                ros::Duration(0.1).sleep();
                
                if (flight_state_.mavros_state.armed) {
                    ROS_INFO("Vehicle successfully armed!");
                    armed_success = true;
                    break;
                }
            }
            
            if (armed_success) break;
        }
        
        ROS_WARN("Arm attempt %d failed, retrying...", i+1);
    }
    
    if (!armed_success) {
        ROS_ERROR("Failed to arm vehicle");
        return;
    }
    
    // 7. 起飞（相对 mission0 原点）
    if (!path_layers_.empty()) {
        double layer_approach_z = path_layers_[0].z_center + 0.5;
        takeoff_target_z = std::max(takeoff_target_z, layer_approach_z);
    }

    if (!takeoffToMapZ(takeoff_target_z)) {
        ROS_ERROR("Takeoff failed");
        return;
    }
    
       
    // 8. 等待2秒让无人机稳定（同时持续发送hold指令）
    //    注意：先稳定再生成轨迹，确保起点位置准确
    ROS_INFO("Waiting for drone to stabilize...");
    ros::Time stabilize_start = ros::Time::now();
    double stabilize_duration = 2.0;

    mavros_msgs::PositionTarget stabilize_cmd;
    stabilize_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    stabilize_cmd.type_mask = 
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    while (ros::ok() && (ros::Time::now() - stabilize_start).toSec() < stabilize_duration) {
        stabilize_cmd.header.stamp = ros::Time::now();
        stabilize_cmd.position.x = flight_state_.current_pose.pose.position.x;
        stabilize_cmd.position.y = flight_state_.current_pose.pose.position.y;
        stabilize_cmd.position.z = flight_state_.current_pose.pose.position.z;
        
        setpoint_raw_pub_.publish(stabilize_cmd);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Stabilization complete");

    // =========================================================================
    // 9. 生成全局合并B样条轨迹
    //    核心改进：将"接近段 + 所有作业层 + 层间过渡"合并为唯一的一条轨迹
    //    彻底解决层切换时的速度/位置不连续问题
    // =========================================================================

    ROS_INFO("Generating GLOBAL merged B-spline trajectory...");

    // 9.1 获取当前无人机精确状态（作为轨迹起点）
    Eigen::Vector3d current_pos(
        flight_state_.current_pose.pose.position.x,
        flight_state_.current_pose.pose.position.y,
        flight_state_.current_pose.pose.position.z
    );

    // 9.2 提取当前偏航角（用于计算起始法向量）
    tf2::Quaternion q(
        flight_state_.current_pose.pose.orientation.x,
        flight_state_.current_pose.pose.orientation.y,
        flight_state_.current_pose.pose.orientation.z,
        flight_state_.current_pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    ROS_INFO("Global trajectory start: pos=(%.2f, %.2f, %.2f), yaw=%.1f°",
             current_pos.x(), current_pos.y(), current_pos.z(), yaw * 180.0 / M_PI);

    // 9.3 调用PathPlanner生成全局B样条
    //     这个函数内部会完成：
    //       - 接近段：零速度起步 + 平滑引导
    //       - 作业层：重叠延伸闭合避免尖角
    //       - 层间过渡：动态多点插值 + SLERP法向量
    //       - 终点：零速度悬停
    std::vector<double> layer_times; // 新建变量接收层时间标记
    ship_painter::BSpline global_bspline = planner_.generateGlobalBSpline(
        current_pos,
        yaw,
        path_layers_,
        layer_times  // 传入变量
    );

    // 9.4 验证生成结果
    if (global_bspline.getControlPoints().empty()) {
        ROS_ERROR("Failed to generate global trajectory! Control points empty.");
        ROS_ERROR("Aborting mission for safety.");
        return;
    }

    ROS_INFO("✓ Global trajectory generated successfully");
    ROS_INFO("  Total time: %.2f seconds", global_bspline.getTotalTime());
    ROS_INFO("  Control points: %zu", global_bspline.getControlPoints().size());

    // 9.5 存储到bspline_layers_以便可视化（复用原有可视化逻辑）
    //     虽然名字叫"layers"，但实际只存一个元素
    bspline_layers_.clear();
    PathPlanner::BSplineLayer global_layer_wrapper;
    global_layer_wrapper.trajectory = global_bspline;
    global_layer_wrapper.layer_index = 0;
    global_layer_wrapper.z_height = 0.0; // 全局轨迹无单一高度
    bspline_layers_.push_back(global_layer_wrapper);
    bsplines_generated_ = true;


    // 10. 发布全局轨迹到 trajectory_server

    ROS_INFO("Publishing global trajectory to trajectory_server...");
    ship_painter::BsplineLayer msg;

    // 构建ROS消息
    ship_painter::Bspline bspline_msg;
    bspline_msg.order = global_bspline.getDegree();
    bspline_msg.start_time = ros::Time::now() + ros::Duration(1.0);  // 1秒后启动
    bspline_msg.duration = global_bspline.getTotalTime();

    // 填充控制点
    for (const auto& pt : global_bspline.getControlPoints()) {
        geometry_msgs::Point p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = pt.z();
        bspline_msg.pos_pts.push_back(p);
    }

    // 填充节点向量
    bspline_msg.knots = global_bspline.getKnots();

    // 填充法向量
    for (const auto& n : global_bspline.getNormals()) {
        geometry_msgs::Vector3 vn;
        vn.x = n.x();
        vn.y = n.y();
        vn.z = n.z();
        bspline_msg.normals.push_back(vn);
    }

    msg.layers.push_back(bspline_msg);  // 只有一层！

    // 填充层时间标记
    msg.layer_end_times = layer_times;

    // 发布
    bspline_pub_.publish(msg);
    ROS_INFO("Global trajectory published (1 unified trajectory, %zu control points, %zu layer markers)",
             global_bspline.getControlPoints().size(), layer_times.size());

    
    ROS_INFO("Maintaining setpoint stream, waiting for trajectory_server to take over...");

    received_trajectory_setpoint_ = false;  // 重置标志
    ros::Time handover_start = ros::Time::now();
    double handover_timeout = 2.0;  // 最多等2秒，防止卡死

    mavros_msgs::PositionTarget hold_cmd;
    hold_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    hold_cmd.type_mask = 
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    while (ros::ok() && !received_trajectory_setpoint_) {
        // 超时检查
        if ((ros::Time::now() - handover_start).toSec() > handover_timeout) {
            ROS_WARN("Handover timeout! trajectory_server may not be running properly");
            break;
        }
        
        hold_cmd.header.stamp = ros::Time::now();
        hold_cmd.position.x = flight_state_.current_pose.pose.position.x;
        hold_cmd.position.y = flight_state_.current_pose.pose.position.y;
        hold_cmd.position.z = flight_state_.current_pose.pose.position.z;
        
        setpoint_raw_pub_.publish(hold_cmd);
        ros::spinOnce();  // 处理回调，接收trajectory_server的setpoint
        rate.sleep();
    }

    if (received_trajectory_setpoint_) {
        ROS_INFO("Handover complete, trajectory_server is now in control");
    } else {
        ROS_ERROR("Handover failed! Continuing anyway...");
    }
    
    // 12. 等待轨迹完成
    ROS_INFO("Control handed to trajectory_server. Waiting for completion...");

    ros::Time mission_start = ros::Time::now();
    double mission_timeout = 300.0;  // 5分钟超时

    while (ros::ok()) {
        //只等待，不发送控制指令
        
        if ((ros::Time::now() - mission_start).toSec() > mission_timeout) {
            ROS_WARN("Mission timeout, landing...");
            break;
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    bool mission_success = true;
    
    // 降落
    if (mission_success) {
        ROS_INFO("Mission successful! Landing...");
    } else {
        ROS_WARN("Mission interrupted. Landing...");
    }
    
    setMode("AUTO.LAND");
    ros::Duration(10.0).sleep();
    
    // 上锁
    disarm();
    ROS_INFO("Mission ended. B-spline trajectory execution complete.");

}



int main(int argc, char** argv) {
    ros::init(argc, argv, "ship_painter_node");
    
    try {
        ShipPainterNode node;
        node.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }
    
    return 0;
}

// =========================================================================
// 统一坐标部署：计算部署平移量
// Level A: 假定当前模型局部坐标中 x_min 面为靠近无人机的一侧。
// 若更换模型需确认此假设，或升级为 Level B（引入 near_face_axis 参数）。
// =========================================================================
void ShipPainterNode::computeDeploymentTransform(
    const Eigen::Vector3d& bbox_min,
    const Eigen::Vector3d& bbox_max,
    Eigen::Vector3d& t_m_out)
{
    double x_ref = bbox_min.x();  // Level A: 近侧 = x_min
    double y_c = (bbox_min.y() + bbox_max.y()) / 2.0;
    double z_min = bbox_min.z();

    t_m_out.x() = params_.mission_forward_distance - x_ref;
    t_m_out.y() = -y_c;
    t_m_out.z() = (-params_.mission_fcu_ground_height + params_.mission_bottom_clearance) - z_min;

    ROS_INFO("Deployment transform t_m: [%.3f, %.3f, %.3f]",
             t_m_out.x(), t_m_out.y(), t_m_out.z());
    ROS_INFO("  x_ref=%.3f, y_c=%.3f, z_min=%.3f", x_ref, y_c, z_min);
}

// =========================================================================
// 统一坐标部署：将所有几何数据从局部系变换到map系
// p_map = o_m + R_m * (p_local + t_m)
// n_map = R_m * n_local
// 姿态基于变换后的法向量重新计算
// =========================================================================
void ShipPainterNode::applyMissionTransform(
    const Eigen::Matrix3d& R_m,
    const Eigen::Vector3d& o_m,
    const Eigen::Vector3d& t_m)
{
    // 1. 变换 path_layers_ 中的所有航点
    for (auto& layer : path_layers_) {
        for (auto& wp : layer.waypoints) {
            wp.position = o_m + R_m * (wp.position + t_m);
            wp.surface_normal = R_m * wp.surface_normal;
            wp.approach_dir = R_m * wp.approach_dir;
        }

        // 更新 z_center 到 map 系（R_m 只绕 z 轴旋转，z 分量不变）
        layer.z_center = o_m.z() + (layer.z_center + t_m.z());

        // 重新计算姿态（基于变换后的法向量）
        for (size_t i = 0; i < layer.waypoints.size(); ++i) {
            auto& wp = layer.waypoints[i];

            // yaw/pitch 从 -surface_normal 导出（机头朝向表面）
            Eigen::Vector3d heading = -wp.surface_normal;
            heading.normalize();

            wp.yaw = atan2(heading.y(), heading.x());

            double horizontal_dist = sqrt(heading.x() * heading.x() + heading.y() * heading.y());
            wp.pitch = atan2(-heading.z(), horizontal_dist);
            const double max_pitch = M_PI / 6;
            wp.pitch = std::max(-max_pitch, std::min(max_pitch, wp.pitch));

            // roll: 横向移动补偿
            wp.roll = 0.0;
            if (i > 0) {
                const auto& prev = layer.waypoints[i - 1];
                Eigen::Vector3d movement = wp.position - prev.position;
                movement.z() = 0;
                if (movement.norm() > 1e-6) {
                    Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(wp.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                    Eigen::Matrix3d R_pitch = Eigen::AngleAxisd(wp.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
                    Eigen::Matrix3d R_body = R_yaw * R_pitch;
                    Eigen::Vector3d movement_body = R_body.transpose() * movement;
                    const double max_roll = M_PI / 12;
                    double roll = std::atan2(movement_body.y(), movement.norm()) * 0.5;
                    wp.roll = std::max(-max_roll, std::min(max_roll, roll));
                }
            }
        }
    }

    // 2. 变换轮廓缓存（从planner获取局部系数据，变换后存到节点侧）
    map_alphashape_contours_.clear();
    for (const auto& [z, contour] : planner_.getAlphaShapeContours()) {
        std::vector<Eigen::Vector3d> transformed;
        transformed.reserve(contour.size());
        for (const auto& pt : contour) {
            transformed.push_back(o_m + R_m * (pt + t_m));
        }
        map_alphashape_contours_.push_back({z, transformed});
    }

    map_offset_contours_.clear();
    for (const auto& [z, contour] : planner_.getOffsetContours()) {
        std::vector<Eigen::Vector3d> transformed;
        transformed.reserve(contour.size());
        for (const auto& pt : contour) {
            transformed.push_back(o_m + R_m * (pt + t_m));
        }
        map_offset_contours_.push_back({z, transformed});
    }

    ROS_INFO("Mission transform applied: %zu layers, %zu alphashape contours, %zu offset contours",
             path_layers_.size(), map_alphashape_contours_.size(), map_offset_contours_.size());
}


void ShipPainterNode::reorderLayersByProximity() {
    if (path_layers_.empty()) {
        ROS_WARN("No path layers to reorder");
        return;
    }
    
    if (!flight_state_.pose_received) {
        ROS_WARN("Drone pose not received, cannot reorder");
        return;
    }
    
    // 获取无人机位置
    Eigen::Vector3d drone_pos(
        flight_state_.current_pose.pose.position.x,
        flight_state_.current_pose.pose.position.y,
        flight_state_.current_pose.pose.position.z
    );
    
    // 按z坐标从大到小排序层（最高层在前）
    std::sort(path_layers_.begin(), path_layers_.end(),
              [](const PathPlanner::PathLayer& a, const PathPlanner::PathLayer& b) {
                  return a.z_center > b.z_center;
              });
    
    ROS_INFO("Layer reordering: %zu layers, %s direction", 
         path_layers_.size(), 
         params_.traverse_clockwise ? "Clockwise" : "Counter-clockwise");
    
    // 辅助函数：计算层中心
    auto computeLayerCenter = [](const std::vector<PathPlanner::Waypoint>& waypoints) -> Eigen::Vector3d {
        Eigen::Vector3d center(0, 0, 0);
        for (const auto& wp : waypoints) {
            center += wp.position;
        }
        return center / waypoints.size();
    };
    
    // 辅助函数：按角度排序航点（从+Z向下看）
    auto reorderByAngle = [](std::vector<PathPlanner::Waypoint>& waypoints, 
                            const Eigen::Vector3d& center, 
                            size_t start_idx,
                            bool clockwise) {
        if (waypoints.empty()) return;
        
        // 计算所有点相对于中心的角度
        std::vector<std::pair<double, size_t>> angles;
        for (size_t i = 0; i < waypoints.size(); ++i) {
            Eigen::Vector3d rel = waypoints[i].position - center;
            double angle = atan2(rel.y(), rel.x());
            angles.push_back({angle, i});
        }
        
        // 找到起始点的角度
        double start_angle = angles[start_idx].first;
        
        // 调整所有角度，使起始点角度为0
        for (auto& ap : angles) {
            ap.first -= start_angle;
            // 归一化到 [0, 2π)
            while (ap.first < 0) ap.first += 2 * M_PI;
            while (ap.first >= 2 * M_PI) ap.first -= 2 * M_PI;
        }
        
        // 根据方向排序
        if (clockwise) {
            // 顺时针：从+Z向下看，角度递减
            // 但我们的angle是从0开始的，所以用负角度排序
            std::sort(angles.begin(), angles.end(),
                     [](const std::pair<double, size_t>& a, const std::pair<double, size_t>& b) {
                         return a.first < b.first;  // 小角度在前
                     });
        } else {
            // 逆时针：角度递增
            std::sort(angles.begin(), angles.end(),
                     [](const std::pair<double, size_t>& a, const std::pair<double, size_t>& b) {
                         return a.first < b.first;
                     });
        }
        
        // 重新排序航点
        std::vector<PathPlanner::Waypoint> reordered;
        for (const auto& ap : angles) {
            reordered.push_back(waypoints[ap.second]);
        }
        waypoints = reordered;
    };
    
    // 处理第一层（最高层）
    if (!path_layers_[0].waypoints.empty()) {
        auto& first_layer = path_layers_[0].waypoints;
        
        // 找到离无人机最近的点
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < first_layer.size(); ++i) {
            double dist = (first_layer[i].position - drone_pos).norm();
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        
        // 计算层中心
        Eigen::Vector3d layer_center = computeLayerCenter(first_layer);
        
        // 按全局方向排序
        reorderByAngle(first_layer, layer_center, closest_idx, params_.traverse_clockwise);
        
        ROS_INFO("Layer 0 reordered: start from closest point (was index %zu), direction: %s",
                 closest_idx, params_.traverse_clockwise ? "clockwise" : "counterclockwise");
    }
    
    // 处理后续层
    for (size_t layer_idx = 1; layer_idx < path_layers_.size(); ++layer_idx) {
        auto& current_layer = path_layers_[layer_idx].waypoints;
        if (current_layer.empty()) continue;
        
        // 获取上一层的第一个点（闭合点）
        const auto& prev_layer_first_point = path_layers_[layer_idx - 1].waypoints[0].position;
        
        // 找到离上一层第一个点最近的点
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < current_layer.size(); ++i) {
            double dist = (current_layer[i].position - prev_layer_first_point).norm();
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        
        // 计算层中心
        Eigen::Vector3d layer_center = computeLayerCenter(current_layer);
        
        // 按全局方向排序
        reorderByAngle(current_layer, layer_center, closest_idx, params_.traverse_clockwise);
        
        // ROS_INFO("Layer %zu reordered: start from point closest to prev layer (was index %zu)",
        //          layer_idx, closest_idx);
    }
    
    ROS_INFO("All %zu layers reordered successfully", path_layers_.size());
}

// 生成从当前位置到第一层的B样条接近轨迹
void ShipPainterNode::generateApproachTrajectory() {
    if (bspline_layers_.empty()) {
        ROS_ERROR("No B-spline layers available for approach trajectory");
        return;
    }
    
    // 获取当前位置
    Eigen::Vector3d current_pos(
        flight_state_.current_pose.pose.position.x,
        flight_state_.current_pose.pose.position.y,
        flight_state_.current_pose.pose.position.z
    );
    
    
    //获取当前偏航角并计算起始法向量
    tf2::Quaternion q(
        flight_state_.current_pose.pose.orientation.x,
        flight_state_.current_pose.pose.orientation.y,
        flight_state_.current_pose.pose.orientation.z,
        flight_state_.current_pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 根据当前 Yaw 计算“虚拟”法向量
    // 原理：yaw 是机头方向，法向量 n 应该是机头反方向 (-heading)
    // 这样 TrajectoryServer 中的 spray_dir = -n 就会还原回 heading
    Eigen::Vector3d start_normal(-cos(yaw), -sin(yaw), 0.0);
    
    // 获取第一层的第一个控制点
    const auto& first_layer = bspline_layers_[0];
    const auto& control_points = first_layer.trajectory.getControlPoints();
    
    if (control_points.empty()) {
        ROS_ERROR("First layer has no control points");
        return;
    }
    
    Eigen::Vector3d target_pos = control_points[0];
    
    // 获取第一层第一个点的法向量（终点法向量）
    Eigen::Vector3d target_normal;
    const auto& first_normals = first_layer.trajectory.getNormals();
    if (!first_normals.empty()) {
        target_normal = first_normals[0];
    } else {
        target_normal = Eigen::Vector3d(0, 0, -1);
    }

    ROS_INFO("Generating approach trajectory...");
    
    std::vector<Eigen::Vector3d> transition_points;
    std::vector<Eigen::Vector3d> transition_normals;
    
    // 1. 起点
    transition_points.push_back(current_pos);
    transition_normals.push_back(start_normal); // 使用当前朝向对应的法向量
    
    // 2. 中间点1 (1/3处)
    transition_points.push_back(current_pos + (target_pos - current_pos) * 0.33);
    // 法向量线性插值 (start -> target)
    Eigen::Vector3d n1 = (start_normal * 0.66 + target_normal * 0.33).normalized();
    transition_normals.push_back(n1);

    // 3. 中间点2 (2/3处)
    transition_points.push_back(current_pos + (target_pos - current_pos) * 0.66);
    // 法向量线性插值
    Eigen::Vector3d n2 = (start_normal * 0.33 + target_normal * 0.66).normalized();
    transition_normals.push_back(n2);
    
    // 4. 终点
    transition_points.push_back(target_pos);
    transition_normals.push_back(target_normal);
    
    // 拟合接近轨迹
    approach_trajectory_.fitFromContour(
        transition_points,
        transition_normals,
        params_.flight_speed, 
        false  // 不闭合
    );
    
    has_approach_trajectory_ = true;
    
    double distance = (target_pos - current_pos).norm();
    ROS_INFO("Approach trajectory generated: %.2f meters, %.2f seconds @ %.2f m/s",
             distance, approach_trajectory_.getTotalTime(), params_.flight_speed);
}

void ShipPainterNode::trajectorySetpointCallback(
    const mavros_msgs::PositionTarget::ConstPtr& msg) {
    
    // 只在等待交接期间处理
    if (!received_trajectory_setpoint_) {
        received_trajectory_setpoint_ = true;
        ROS_INFO("Received first setpoint from trajectory_server, handover confirmed");
    }
}
