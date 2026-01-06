#include <ros/ros.h>
#include "ship_painter/path_planner.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>  // 使用PositionTarget替代AttitudeTarget
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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
    
    // 状态管理
    struct FlightState {
        mavros_msgs::State mavros_state;
        geometry_msgs::PoseStamped current_pose;
        geometry_msgs::PoseStamped home_pose;
        bool pose_received = false;
        bool home_set = false;

        bool path_ready = false;
        
        //路径变换信息
        Eigen::Vector3d path_translation = Eigen::Vector3d::Zero();
        double path_z_offset = 0.0;
        bool path_transformed = false;
        
    } flight_state_;
    
    // 参数配置
    struct Parameters {
        std::string model_path;
        std::string frame_id = "world";
        std::string mavros_ns = "/mavros";//话题名称，虚拟环境加组号iris_0
        
        // 路径位置
        double model_x = 5.0;
        double model_y = 0.0;
        double model_z = 0.0;

            // rviz显示用的模型位置
        double display_model_x = 4;
        double display_model_y = 0;
        double display_model_z = 1;
        
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

        double path_offset_distance = 3.0;//默认模型位置
        bool traverse_clockwise = true;  // 遍历方向：true=顺时针，false=逆时针
    } params_;
    
    // 回调函数
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void visualizationTimerCallback(const ros::TimerEvent& event);

    void trajectorySetpointCallback(const mavros_msgs::PositionTarget::ConstPtr& msg);//交接位

    
    // 飞行控制
    bool waitForConnection();
    bool setMode(const std::string& mode);
    bool arm();
    bool disarm();
    bool takeoff(double height);
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
    //航点迁移到真实世界
    void transformPathRelativeToDrone(double offset_distance);
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
    
    nh_private_.param<double>("model_x", params_.model_x, 5.0);
    nh_private_.param<double>("model_y", params_.model_y, 0.0);
    nh_private_.param<double>("model_z", params_.model_z, 0.0);
    
    nh_private_.param<double>("spray_radius", params_.spray_radius, SPRAY_RADIUS);
    nh_private_.param<double>("spray_distance", params_.spray_distance, SPRAY_DISTANCE);
    nh_private_.param<double>("layer_height", params_.layer_height, SPRAY_WIDTH);
    
    nh_private_.param<double>("takeoff_height", params_.takeoff_height, 2.0);
    nh_private_.param<double>("flight_speed", params_.flight_speed, 5);
    nh_private_.param<double>("approach_speed", params_.approach_speed, 0.3);
    nh_private_.param<double>("transition_speed", params_.transition_speed, 0.3);//B样条速度

    
    nh_private_.param<bool>("use_position_control", params_.use_position_control, false);
    nh_private_.param<bool>("enable_spray_visualization", params_.enable_spray_visualization, true);

    double alpha_value;
    nh_private_.param<double>("alpha_shape_value", alpha_value, 3.0);
    
    int sample_count;
    nh_private_.param<int>("target_sample_count", sample_count, 10000);
    
    nh_private_.param<double>("display_model_x", params_.display_model_x, 4.0);
    nh_private_.param<double>("display_model_y", params_.display_model_y, 0.0);
    nh_private_.param<double>("display_model_z", params_.display_model_z, 1.0);

    nh_private_.param("path_offset_distance", params_.path_offset_distance, 3.0);//真实世界中模型在无人机前方几米处
    nh_private_.param("traverse_clockwise", params_.traverse_clockwise, true);  // 遍历方向
    double resample_spacing_param;
    nh_private_.param<double>("resample_spacing", resample_spacing_param, 0.02); // 步长

    nh_private_.param<std::string>("mavros_ns", params_.mavros_ns, "/mavros");//命名空间，真机/虚拟
    
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
    ROS_INFO("Model position: (%.2f, %.2f, %.2f)", 
             params_.model_x, params_.model_y, params_.model_z);
    ROS_INFO("Spray radius: %.3f m", params_.spray_radius);
    ROS_INFO("Spray distance: %.3f m", params_.spray_distance);
    ROS_INFO("Layer height: %.3f m", params_.layer_height);
    ROS_INFO("Max height diff per layer: %.3f m", MAX_HEIGHT_DIFF);

    ROS_INFO("Flight speed parameter: %.2f m/s", params_.flight_speed);
    
    return true;
}

bool ShipPainterNode::generateSprayPath() {
    ROS_INFO("Generating spray painting path...");
    
    if (!planner_.loadSTLModel(params_.model_path)) {
        ROS_ERROR("Failed to load STL model");
        return false;
    }
    
    Eigen::Vector3d model_position(params_.model_x, params_.model_y, params_.model_z);
    path_layers_ = planner_.generateSprayPath(model_position);
    
    if (path_layers_.empty()) {
        ROS_ERROR("No path layers generated");
        return false;
    }

    ros::Rate wait_rate(10);
    int wait_count = 0;
    while (!flight_state_.pose_received && wait_count < 50) {
        ros::spinOnce();
        wait_rate.sleep();
        wait_count++;
    }
    
    if (flight_state_.pose_received) {
        transformPathRelativeToDrone(params_.path_offset_distance);  // 3m偏移
        reorderLayersByProximity();
    } else {
        ROS_WARN("Drone pose not available, using original path");
    }
    
    flight_state_.path_ready = true;
    // printMissionSummary();//不再汇总打印
    
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

void ShipPainterNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    flight_state_.current_pose = *msg;
    
    if (!flight_state_.pose_received) {
        flight_state_.pose_received = true;
        ROS_INFO("First pose received");
    }
    
    if (!flight_state_.home_set) {
        flight_state_.home_pose = *msg;
        flight_state_.home_set = true;
        ROS_INFO("Home position set: (%.2f, %.2f, %.2f)",
                 flight_state_.home_pose.pose.position.x,
                 flight_state_.home_pose.pose.position.y,
                 flight_state_.home_pose.pose.position.z);
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
    auto alphashape_contours = planner_.getAlphaShapeContours();
    if (alphashape_contours.empty()) return;
    
    visualization_msgs::MarkerArray marker_array;
    
    for (size_t i = 0; i < alphashape_contours.size(); ++i) {
        const auto& layer_data = alphashape_contours[i];
        double z_center = layer_data.first;
        const auto& contour = layer_data.second;
        
        if (contour.size() < 2) continue;
        
        visualization_msgs::Marker contour_marker;
        contour_marker.header.frame_id = "map";
        contour_marker.header.stamp = ros::Time::now();
        contour_marker.ns = "alphashape_contour";
        contour_marker.id = i;
        contour_marker.type = visualization_msgs::Marker::LINE_STRIP;
        contour_marker.action = visualization_msgs::Marker::ADD;
        contour_marker.scale.x = 0.03; // 稍粗一些以便区分
        
        // 红色显示原始Alpha Shape轮廓
        contour_marker.color.r = 1.0;
        contour_marker.color.g = 0.0;
        contour_marker.color.b = 0.0;
        contour_marker.color.a = 0.8;
        
        for (const auto& pt : contour) {
            geometry_msgs::Point p;
            //变换
            Eigen::Vector3d transformed_pt = pt;
            if (flight_state_.path_transformed) {
                transformed_pt += flight_state_.path_translation;
                transformed_pt.z() += flight_state_.path_z_offset;
            }
            
            p.x = transformed_pt.x();
            p.y = transformed_pt.y();
            p.z = transformed_pt.z();
            contour_marker.points.push_back(p);
        }
        
        // 确保轮廓闭合
        if (contour.size() > 2) {
            geometry_msgs::Point first_pt;
            
            //应用变换
            Eigen::Vector3d transformed_pt = contour[0];
            if (flight_state_.path_transformed) {
                transformed_pt += flight_state_.path_translation;
                transformed_pt.z() += flight_state_.path_z_offset;
            }
            
            first_pt.x = transformed_pt.x();
            first_pt.y = transformed_pt.y();
            first_pt.z = transformed_pt.z();
            contour_marker.points.push_back(first_pt);
        }
        
        marker_array.markers.push_back(contour_marker);
    }
    
    alphashape_vis_pub_.publish(marker_array);
}

void ShipPainterNode::publishOffsetContourVisualization() {
    auto offset_contours = planner_.getOffsetContours();
    if (offset_contours.empty()) return;
    
    visualization_msgs::MarkerArray marker_array;
    
    // ROS_INFO_THROTTLE(5, "Publishing %zu offset contours for visualization", offset_contours.size());
    
    for (size_t i = 0; i < offset_contours.size(); ++i) {
        const auto& layer_data = offset_contours[i];
        double z_center = layer_data.first;
        const auto& contour = layer_data.second;
        
        if (contour.size() < 2) continue;
        
        // 偏移轮廓线条（蓝色）
        visualization_msgs::Marker contour_marker;
        contour_marker.header.frame_id = "map";
        contour_marker.header.stamp = ros::Time::now();
        contour_marker.ns = "offset_contour";
        contour_marker.id = i;
        contour_marker.type = visualization_msgs::Marker::LINE_STRIP;
        contour_marker.action = visualization_msgs::Marker::ADD;
        contour_marker.scale.x = 0.04; // 比原始轮廓稍粗
        
        // 蓝色显示偏移后的轮廓
        contour_marker.color.r = 0.0;
        contour_marker.color.g = 0.4;  // 稍微有点绿色调
        contour_marker.color.b = 1.0;  // 主要是蓝色
        contour_marker.color.a = 0.9;  // 比较不透明，突出显示
        
        // 添加所有轮廓点
        for (const auto& pt : contour) {
            geometry_msgs::Point p;
            
            //应用变换
            Eigen::Vector3d transformed_pt = pt;
            if (flight_state_.path_transformed) {
                transformed_pt += flight_state_.path_translation;
                transformed_pt.z() += flight_state_.path_z_offset;
            }
            
            p.x = transformed_pt.x();
            p.y = transformed_pt.y();
            p.z = transformed_pt.z() + 0.01;
            contour_marker.points.push_back(p);
        }

        // 确保轮廓闭合
        if (contour.size() > 2) {
            geometry_msgs::Point first_pt;
            
            // 应用变换到闭合点
            Eigen::Vector3d transformed_pt = contour[0];
            if (flight_state_.path_transformed) {
                transformed_pt += flight_state_.path_translation;
                transformed_pt.z() += flight_state_.path_z_offset;
            }
            
            first_pt.x = transformed_pt.x();
            first_pt.y = transformed_pt.y();
            first_pt.z = transformed_pt.z() + 0.01;
            contour_marker.points.push_back(first_pt);
        }

        marker_array.markers.push_back(contour_marker);
        
        // 添加轮廓点标记（蓝色小球）
        visualization_msgs::Marker points_marker;
        points_marker.header = contour_marker.header;
        points_marker.ns = "offset_contour_points";
        points_marker.id = i + 1000;
        points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        points_marker.action = visualization_msgs::Marker::ADD;
        points_marker.scale.x = 0.02;  // 2cm直径的小球
        points_marker.scale.y = 0.02;
        points_marker.scale.z = 0.02;
        
        // 稍微深一点的蓝色
        points_marker.color.r = 0.0;
        points_marker.color.g = 0.2;
        points_marker.color.b = 0.8;
        points_marker.color.a = 0.8;
        
        // 采样显示点（避免过于密集）
        size_t step = std::max(1UL, contour.size() / 50);
        for (size_t j = 0; j < contour.size(); j += step) {
            geometry_msgs::Point p;
            
            //应用变换到球体点
            Eigen::Vector3d transformed_pt = contour[j];
            if (flight_state_.path_transformed) {
                transformed_pt += flight_state_.path_translation;
                transformed_pt.z() += flight_state_.path_z_offset;
            }
            
            p.x = transformed_pt.x();
            p.y = transformed_pt.y();
            p.z = transformed_pt.z() + 0.02;
            points_marker.points.push_back(p);
        }

        marker_array.markers.push_back(points_marker);
        
        // 添加层标签
        visualization_msgs::Marker text_marker;
        text_marker.header = contour_marker.header;
        text_marker.ns = "offset_contour_labels";
        text_marker.id = i + 2000;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        // 在轮廓中心显示层信息
        Eigen::Vector3d center = computeContourCenter(contour);
        //应用变换到中心点
        if (flight_state_.path_transformed) {
            center += flight_state_.path_translation;
            center.z() += flight_state_.path_z_offset;
        }

        text_marker.pose.position.x = center.x();
        text_marker.pose.position.y = center.y();
        text_marker.pose.position.z = center.z() + 0.1; // 高一点显示文字
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.scale.z = 0.1; // 文字大小
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
    
    // 调试信息
    // static int debug_count = 0;
    // if (++debug_count % 100 == 0) {
    //     ROS_INFO("Published offset contour visualization: %zu layers, %zu markers total",
    //              offset_contours.size(), marker_array.markers.size());
    // }
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
    visualization_msgs::Marker model_marker;
    model_marker.header.frame_id = "map";
    model_marker.header.stamp = ros::Time::now();
    model_marker.ns = "ship_model";
    model_marker.id = 0;
    model_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    model_marker.mesh_resource = "file://" + params_.model_path;
    model_marker.action = visualization_msgs::Marker::ADD;
    
    model_marker.pose.position.x = params_.display_model_x;
    model_marker.pose.position.y = params_.display_model_y;
    model_marker.pose.position.z = params_.display_model_z;
    model_marker.pose.orientation.w = 1.0;
    
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
    
    status_marker.pose.position.x = params_.model_x;
    status_marker.pose.position.y = params_.model_y;
    status_marker.pose.position.z = params_.model_z + 2.0;
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
    processed_marker.scale.x = 0.015;  // 1.5cm直径的小球
    processed_marker.scale.y = 0.015;
    processed_marker.scale.z = 0.015;
    
    // 设置颜色为白色
    processed_marker.color.r = 1.0;
    processed_marker.color.g = 1.0;
    processed_marker.color.b = 1.0;
    processed_marker.color.a = 0.8;   ///透明度
    
    // 添加所有点
    for (const auto& point : processed_cloud->points) {
        // 跳过无效点
        if (!pcl::isFinite(point)) {
            continue;
        }
        
        geometry_msgs::Point p;
        p.x = point.x + params_.display_model_x;  // 应用模型位置偏移
        p.y = point.y + params_.display_model_y;
        p.z = point.z + params_.display_model_z;
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
        size_t step = std::max(1UL, original_cloud->size() / 20000);  // 最多显示10000个点
        for (size_t i = 0; i < original_cloud->size(); i += step) {
            const auto& point = original_cloud->points[i];
            if (!pcl::isFinite(point)) {
                continue;
            }
            
            geometry_msgs::Point p;
            p.x = point.x + params_.display_model_x;
            p.y = point.y + params_.display_model_y;
            p.z = point.z + params_.display_model_z;
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
        
        // 应用模型位置偏移
        center.x() += params_.model_x;
        center.y() += params_.model_y;
        center.z() += params_.model_z;
        
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

            geometry_msgs::Point start, end;
            start.x = point.x + params_.model_x;
            start.y = point.y + params_.model_y;
            start.z = point.z + params_.model_z;

            Eigen::Vector3d end_e = Eigen::Vector3d(start.x, start.y, start.z) + n.normalized() * normal_length;
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

bool ShipPainterNode::takeoff(double height) {
    ROS_INFO("Taking off to %.1f meters...", height);
    
    geometry_msgs::PoseStamped takeoff_pose = flight_state_.current_pose;
    takeoff_pose.pose.position.z = height;
    
    ros::Rate rate(20);
    while (ros::ok()) {
        local_pos_pub_.publish(takeoff_pose);
        
        double current_height = flight_state_.current_pose.pose.position.z;
        if (std::abs(current_height - height) < 0.2) {
            ROS_INFO("Takeoff complete at %.2f meters", current_height);
            return true;
        }
        
        ROS_INFO_THROTTLE(1, "Taking off... Current: %.2f, Target: %.2f", 
                          current_height, height);
        
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
    ROS_INFO("Waiting for position data and EKF2 initialization...");
    ros::Rate rate(20);
    ros::Time ekf_wait_start = ros::Time::now();
    bool ekf_ready = false;

    while (ros::ok() && !ekf_ready) {
        ros::spinOnce();
        
        if (flight_state_.pose_received) {
            // 检查高度是否合理（应该接近0，允许±.5米的误差）
            double current_height = flight_state_.current_pose.pose.position.z;
            if (fabs(current_height) < 1.5) {
                ekf_ready = true;
                ROS_INFO("EKF2 initialized! Current position: (%.2f, %.2f, %.2f)", 
                        flight_state_.current_pose.pose.position.x,
                        flight_state_.current_pose.pose.position.y,
                        flight_state_.current_pose.pose.position.z);
            } else {
                // 每5秒打印一次当前高度
                static ros::Time last_warn = ros::Time::now();
                if ((ros::Time::now() - last_warn).toSec() > 5.0) {
                    ROS_WARN("Waiting for EKF2... Current height: %.2f m (should be near 0)", 
                            current_height);
                    last_warn = ros::Time::now();
                }
            }
        }
        
        // 超时检查（30秒）
        if ((ros::Time::now() - ekf_wait_start).toSec() > 30.0) {
            ROS_ERROR("EKF2 initialization timeout! Current height: %.2f m", 
                    flight_state_.current_pose.pose.position.z);
            ROS_ERROR("Please check:");
            ROS_ERROR("  1. Is the drone on a flat surface?");
            ROS_ERROR("  2. Is GPS signal good?");
            ROS_ERROR("  3. Try increasing the launch delay time.");
            return;
        }
        
        rate.sleep();
    }

    ROS_INFO("First pose received");
    
    // 3. 生成喷涂路径
    if (!generateSprayPath()) {
        ROS_ERROR("Failed to generate spray path");
        return;
    }
    
    // 4. 发送初始设定点
    ROS_INFO("Sending initial setpoints...");
    geometry_msgs::PoseStamped init_pose = flight_state_.home_pose;
    init_pose.pose.position.z = params_.takeoff_height;
    
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
    
    // 7. 起飞
    double first_layer_height = path_layers_.empty() ? 
                               params_.takeoff_height : 
                               path_layers_[0].z_center + 0.5;  // 稍高一点以便安全接近
    
    if (!takeoff(first_layer_height)) {
        ROS_ERROR("Takeoff failed");
        return;
    }
    
       
    // 8. 生成B样条轨迹
    ROS_INFO("Generating B-spline trajectories...");
    publishBSplineTrajectories();
    
    // 9. 生成接近轨迹
    ROS_INFO("Generating approach trajectory...");
    generateApproachTrajectory();
    
    if (!has_approach_trajectory_) {
        ROS_ERROR("Failed to generate approach trajectory");
        return;
    }
    
    // 10. 等待2秒让无人机稳定（同时持续发送hold指令）
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
    
    // 11. 构建并发布完整的B样条轨迹消息（接近轨迹 + 正式轨迹）
    ROS_INFO("Publishing complete trajectory to trajectory_server...");
    ship_painter::BsplineLayer msg;
    
    // 11.1 第一层：接近轨迹
    ship_painter::Bspline approach_msg;
    approach_msg.order = approach_trajectory_.getDegree();
    approach_msg.start_time = ros::Time::now() + ros::Duration(1.0);  // 1秒后开始
    approach_msg.duration = approach_trajectory_.getTotalTime();
    
    // 控制点
    for (const auto& pt : approach_trajectory_.getControlPoints()) {
        geometry_msgs::Point p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = pt.z();
        approach_msg.pos_pts.push_back(p);
    }
    
    // 节点向量
    approach_msg.knots = approach_trajectory_.getKnots();
    
    // 法向量
    for (const auto& normal : approach_trajectory_.getNormals()) {
        geometry_msgs::Vector3 n;
        n.x = normal.x();
        n.y = normal.y();
        n.z = normal.z();
        approach_msg.normals.push_back(n);
    }
    
    msg.layers.push_back(approach_msg);  // 添加接近轨迹作为第一层
    
    // 11.2 后续层：正式的喷涂轨迹
    for (size_t i = 0; i < bspline_layers_.size(); i++) {
        const auto& layer = bspline_layers_[i];
        
        ship_painter::Bspline bspline_msg;
        bspline_msg.order = layer.trajectory.getDegree();
        bspline_msg.start_time = ros::Time::now() + ros::Duration(2.0);  // 占位，实际由server控制
        bspline_msg.duration = layer.trajectory.getTotalTime();
        
        // 控制点
        for (const auto& pt : layer.trajectory.getControlPoints()) {
            geometry_msgs::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            bspline_msg.pos_pts.push_back(p);
        }
        
        // 节点向量
        bspline_msg.knots = layer.trajectory.getKnots();
        
        // 法向量
        for (const auto& normal : layer.trajectory.getNormals()) {
            geometry_msgs::Vector3 n;
            n.x = normal.x();
            n.y = normal.y();
            n.z = normal.z();
            bspline_msg.normals.push_back(n);
        }
        
        msg.layers.push_back(bspline_msg);
    }
    
    // 11.3 发布完整消息
    bspline_pub_.publish(msg);
    ROS_INFO("Published complete trajectory: 1 approach + %zu work layers = %zu total",
             bspline_layers_.size(), msg.layers.size());
    
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

//无人机在真实世界对应函数
void ShipPainterNode::transformPathRelativeToDrone(double offset_distance) {
    if (!flight_state_.pose_received) {
        ROS_WARN("Drone pose not received, cannot transform path");
        return;
    }
    
    if (path_layers_.empty()) {
        ROS_WARN("No path layers to transform");
        return;
    }
    
    // 获取无人机当前位置
    Eigen::Vector3d drone_pos(
        flight_state_.current_pose.pose.position.x,
        flight_state_.current_pose.pose.position.y,
        flight_state_.current_pose.pose.position.z
    );
    
    // 获取无人机yaw角
    tf2::Quaternion q(
        flight_state_.current_pose.pose.orientation.x,
        flight_state_.current_pose.pose.orientation.y,
        flight_state_.current_pose.pose.orientation.z,
        flight_state_.current_pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // 机头方向单位向量
    Eigen::Vector3d heading_dir(cos(yaw), sin(yaw), 0);
    
    // 第一步：对中无人机（y方向）
    // 计算所有航点的y坐标平均值
    double sum_y = 0.0;
    size_t total_waypoints = 0;
    
    for (const auto& layer : path_layers_) {
        for (const auto& wp : layer.waypoints) {
            sum_y += wp.position.y();
            total_waypoints++;
        }
    }
    
    if (total_waypoints == 0) {
        ROS_WARN("No waypoints found to transform");
        return;
    }
    
    double avg_y = sum_y / total_waypoints;
    double y_offset = avg_y - drone_pos.y();  // 航点集中心相对无人机的y偏移
    
    // 对所有航点的y坐标减去这个偏移量，实现对中
    for (auto& layer : path_layers_) {
        for (auto& wp : layer.waypoints) {
            wp.position.y() -= y_offset;
        }
    }
    
    ROS_INFO("Path centered: y_offset = %.3f m (waypoints avg_y: %.3f, drone_y: %.3f)", 
             y_offset, avg_y, drone_pos.y());
    
    // 第二步：将航点集移动到无人机前方指定距离
    // 找到所有航点中在机头方向上最近的点
    double min_forward_dist = std::numeric_limits<double>::max();
    Eigen::Vector3d closest_waypoint_pos;
    bool found = false;
    
    for (const auto& layer : path_layers_) {
        for (const auto& wp : layer.waypoints) {
            Eigen::Vector3d rel_vec = wp.position - drone_pos;
            double forward_dist = rel_vec.dot(heading_dir);
            
            if (forward_dist < min_forward_dist) {
                min_forward_dist = forward_dist;
                closest_waypoint_pos = wp.position;
                found = true;
            }
        }
    }
    
    if (!found) {
        ROS_WARN("No waypoints found to transform");
        return;
    }
    
    // 计算平移向量
    Eigen::Vector3d target_pos = drone_pos + heading_dir * offset_distance;
    target_pos.z() = closest_waypoint_pos.z();  // 保留原路径高度
    Eigen::Vector3d translation = target_pos - closest_waypoint_pos;
    
    // 对所有航点应用平移
    for (auto& layer : path_layers_) {
        for (auto& wp : layer.waypoints) {
            wp.position += translation;
            wp.position.z() += flight_state_.home_pose.pose.position.z;
        }
    }
    //储存变换信息
    flight_state_.path_translation = translation;
    flight_state_.path_z_offset = flight_state_.home_pose.pose.position.z;
    flight_state_.path_transformed = true;
    
    ROS_INFO("Path transformed: closest waypoint moved to %.2fm forward", offset_distance);
    ROS_INFO("Translation: [%.2f, %.2f, %.2f] m", 
             translation.x(), translation.y(), translation.z());
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

    
    // 获取第一层的第一个控制点
    const auto& first_layer = bspline_layers_[0];
    const auto& control_points = first_layer.trajectory.getControlPoints();
    
    if (control_points.empty()) {
        ROS_ERROR("First layer has no control points");
        return;
    }
    
    Eigen::Vector3d target_pos = control_points[0];
    
    ROS_INFO("Generating approach trajectory from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
             current_pos.x(), current_pos.y(), current_pos.z(),
             target_pos.x(), target_pos.y(), target_pos.z());
    
    // 创建起飞位置的临时层
    PathPlanner::BSplineLayer takeoff_layer;
    
    // 3个控制点：起点 - 中点 - 终点（生成平滑过渡）
    std::vector<Eigen::Vector3d> transition_points;
    std::vector<Eigen::Vector3d> transition_normals;
    
    // 起点：当前位置
    transition_points.push_back(current_pos);
    transition_normals.push_back(Eigen::Vector3d(0, 0, -1));
    
    // 中点：插值位置
    Eigen::Vector3d mid_pos = (current_pos + target_pos) * 0.5;
    transition_points.push_back(mid_pos);
    transition_normals.push_back(Eigen::Vector3d(0, 0, -1));
    
    // 终点：第一层第一个点
    transition_points.push_back(target_pos);
    
    // 获取第一层第一个点的法向量
    const auto& first_normals = first_layer.trajectory.getNormals();
    if (!first_normals.empty()) {
        transition_normals.push_back(first_normals[0]);
    } else {
        transition_normals.push_back(Eigen::Vector3d(0, 0, -1));
    }
    
    // 拟合接近轨迹（不闭合）
    approach_trajectory_.fitFromContour(
        transition_points,
        transition_normals,
        params_.flight_speed,  // 使用设定的飞行速度
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
