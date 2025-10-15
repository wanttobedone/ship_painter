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
    ros::Publisher local_pos_pub_;
    ros::Publisher setpoint_raw_pub_;  // 使用PositionTarget进行混合控制
    ros::Publisher path_vis_pub_;
    ros::Publisher spray_vis_pub_;
    ros::Publisher model_vis_pub_;
    ros::Publisher status_pub_;
    
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;
    
    ros::Timer vis_timer_;
    ros::Timer control_timer_;
    
    // 路径规划器
    PathPlanner planner_;
    std::vector<PathPlanner::PathLayer> path_layers_;

    ros::Publisher pointcloud_vis_pub_;  // 点云可视化发布者   
    ros::Publisher normal_vis_pub_;  // 法向量可视化发布者
    ros::Publisher alphashape_vis_pub_;//as轮廓发布
    ros::Publisher offset_contour_vis_pub_;  // 偏移轮廓可视化发布者
    
    // 状态管理
    struct FlightState {
        mavros_msgs::State mavros_state;
        geometry_msgs::PoseStamped current_pose;
        geometry_msgs::PoseStamped home_pose;
        bool pose_received = false;
        bool home_set = false;
        
        size_t current_layer_idx = 0;
        size_t current_waypoint_idx = 0;
        bool mission_complete = false;
        bool path_ready = false;
        
        // 控制参数
        double position_tolerance = 0.15;  // 位置容差 (m)
        double orientation_tolerance = 0.1; // 姿态容差 (rad)
        double waypoint_hold_time = 0.5;  // 航点保持时间 (s)
        ros::Time waypoint_reached_time;
        bool waypoint_reached = false;
        
        // 平滑过渡
        Eigen::Vector3d last_position;
        Eigen::Quaterniond last_orientation;
        bool has_last_state = false;
        
        // 统计
        size_t total_waypoints_reached = 0;
    } flight_state_;
    
    // 参数配置
    struct Parameters {
        std::string model_path;
        std::string frame_id = "world";
        
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
        double flight_speed = 0.5;        // 巡航速度
        double approach_speed = 0.3;      // 接近速度
        double max_roll = 0.26;           // 最大翻滚角 (15度)
        double max_pitch_adjustment = 0.52; // 最大俯仰调整 (30度)
        
        // 控制模式
        bool use_position_control = false; // 是否使用纯位置控制
        bool enable_spray_visualization = true;
        
        // 姿态控制增益
        double yaw_gain = 0.8;
        double pitch_gain = 0.8;
        double roll_gain = 0.5;
    } params_;
    
    // 回调函数
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void visualizationTimerCallback(const ros::TimerEvent& event);
    void controlTimerCallback(const ros::TimerEvent& event);
    
    // 飞行控制
    bool waitForConnection();
    bool setMode(const std::string& mode);
    bool arm();
    bool disarm();
    bool takeoff(double height);
    bool land();
    
    // 路径跟踪
    bool executeSprayMission();
    void publishControlCommand();
    geometry_msgs::PoseStamped computeTargetPose();
    mavros_msgs::PositionTarget computePositionTarget();
    
    // 姿态计算
    Eigen::Quaterniond computeSprayOrientation(const PathPlanner::Waypoint& waypoint,
                                               const Eigen::Vector3d& current_pos);
    double computeRollForLateralMovement(const Eigen::Vector3d& from,
                                         const Eigen::Vector3d& to,
                                         double current_yaw);
    
    // 航点管理
    bool isWaypointReached();
    void advanceToNextWaypoint();
    PathPlanner::Waypoint getCurrentWaypoint();
    PathPlanner::Waypoint getInterpolatedWaypoint(double alpha);
    
    // 可视化
    void publishPathVisualization();
    void publishSprayVisualization();
    void publishModelVisualization();
    void publishStatusVisualization(); 
    void publishNormalVisualization();  // 法向量可视化
    void publishPointCloudVisualization();//点云可视化
    void publishAlphaShapeVisualization();//as后的路径
    void publishOffsetContourVisualization();//偏移后的路径
    Eigen::Vector3d computeContourCenter(const std::vector<Eigen::Vector3d>& contour);
    
    // 辅助函数
    bool loadParameters();
    bool generateSprayPath();
    void printMissionSummary();
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    double calculateOrientationError(const geometry_msgs::Quaternion& q1, 
                                    const geometry_msgs::Quaternion& q2);
    Eigen::Vector3d smoothPosition(const Eigen::Vector3d& target, 
                                   const Eigen::Vector3d& current, 
                                   double smoothing_factor);
    Eigen::Quaterniond smoothOrientation(const Eigen::Quaterniond& target,
                                         const Eigen::Quaterniond& current,
                                         double smoothing_factor);
};

ShipPainterNode::ShipPainterNode() : nh_private_("~") {
    if (!loadParameters()) {
        ROS_ERROR("Failed to load parameters");
        ros::shutdown();
        return;
    }
    
    // 设置订阅者
    state_sub_ = nh_.subscribe("/iris_0/mavros/state", 10, 
                               &ShipPainterNode::stateCallback, this);
    pose_sub_ = nh_.subscribe("/iris_0/mavros/local_position/pose", 10, 
                             &ShipPainterNode::poseCallback, this);
    
    // 设置发布者
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/iris_0/mavros/setpoint_position/local", 10);
    setpoint_raw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
        "/iris_0/mavros/setpoint_raw/local", 10);
    
    path_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/visualization/path", 10);
    spray_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/visualization/spray", 10);
    model_vis_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/visualization/model", 10);
    status_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/visualization/status", 10);
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
    
    // 设置服务客户端
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
        "/iris_0/mavros/set_mode");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
        "/iris_0/mavros/cmd/arming");
    
    // 设置定时器
    vis_timer_ = nh_.createTimer(ros::Duration(0.1), 
                                 &ShipPainterNode::visualizationTimerCallback, this);
    control_timer_ = nh_.createTimer(ros::Duration(0.05),  // 20Hz控制频率
                                     &ShipPainterNode::controlTimerCallback, this);
    control_timer_.stop();  // 初始停止
    
    ROS_INFO("Ship Painter Node initialized");
}

bool ShipPainterNode::loadParameters() {
    nh_private_.param<std::string>("model_path", params_.model_path, "");
    nh_private_.param<std::string>("frame_id", params_.frame_id, "world");
    
    nh_private_.param<double>("model_x", params_.model_x, 5.0);
    nh_private_.param<double>("model_y", params_.model_y, 0.0);
    nh_private_.param<double>("model_z", params_.model_z, 0.0);
    
    nh_private_.param<double>("spray_radius", params_.spray_radius, SPRAY_RADIUS);
    nh_private_.param<double>("spray_distance", params_.spray_distance, SPRAY_DISTANCE);
    nh_private_.param<double>("layer_height", params_.layer_height, SPRAY_WIDTH);
    
    nh_private_.param<double>("takeoff_height", params_.takeoff_height, 2.0);
    nh_private_.param<double>("flight_speed", params_.flight_speed, 0.5);
    nh_private_.param<double>("approach_speed", params_.approach_speed, 0.3);
    nh_private_.param<double>("position_tolerance", flight_state_.position_tolerance, 0.15);
    nh_private_.param<double>("orientation_tolerance", flight_state_.orientation_tolerance, 0.1);
    
    nh_private_.param<bool>("use_position_control", params_.use_position_control, false);
    nh_private_.param<bool>("enable_spray_visualization", params_.enable_spray_visualization, true);

    double alpha_value;
    nh_private_.param<double>("alpha_shape_value", alpha_value, 3.0);
    
    int sample_count;
    nh_private_.param<int>("target_sample_count", sample_count, 10000);
    
    nh_private_.param<double>("display_model_x", params_.display_model_x, 4.0);
    nh_private_.param<double>("display_model_y", params_.display_model_y, 0.0);
    nh_private_.param<double>("display_model_z", params_.display_model_z, 1.0);
    
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
    
    planner_ = PathPlanner(planner_config);
    
    ROS_INFO("=== Spray Painting Configuration ===");
    ROS_INFO("Model: %s", params_.model_path.c_str());
    ROS_INFO("Model position: (%.2f, %.2f, %.2f)", 
             params_.model_x, params_.model_y, params_.model_z);
    ROS_INFO("Spray radius: %.3f m", params_.spray_radius);
    ROS_INFO("Spray distance: %.3f m", params_.spray_distance);
    ROS_INFO("Layer height: %.3f m", params_.layer_height);
    ROS_INFO("Max height diff per layer: %.3f m", MAX_HEIGHT_DIFF);
    
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
    
    flight_state_.path_ready = true;
    printMissionSummary();
    
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
    
    double estimated_time = total_waypoints * (flight_state_.waypoint_hold_time + 1.0);
    ROS_INFO("Estimated mission time: %.1f seconds", estimated_time);
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

void ShipPainterNode::controlTimerCallback(const ros::TimerEvent& event) {
    if (!flight_state_.path_ready || flight_state_.mission_complete) {
        return;
    }
    
    publishControlCommand();
    
    // 检查是否到达当前航点
    if (isWaypointReached()) {
        if (!flight_state_.waypoint_reached) {
            flight_state_.waypoint_reached = true;
            flight_state_.waypoint_reached_time = ros::Time::now();
            ROS_INFO("Reached waypoint %zu in layer %zu",
                     flight_state_.current_waypoint_idx + 1,
                     flight_state_.current_layer_idx + 1);
        }
        
        // 在航点保持一段时间后前进
        if ((ros::Time::now() - flight_state_.waypoint_reached_time).toSec() > 
            flight_state_.waypoint_hold_time) {
            advanceToNextWaypoint();
        }
    }
}

void ShipPainterNode::publishControlCommand() {
    if (params_.use_position_control) {
        // 纯位置控制模式
        geometry_msgs::PoseStamped pose_target = computeTargetPose();
        local_pos_pub_.publish(pose_target);
    } else {
        // 混合控制模式（位置+姿态+roll）
        mavros_msgs::PositionTarget pos_target = computePositionTarget();
        setpoint_raw_pub_.publish(pos_target);
    }
}

geometry_msgs::PoseStamped ShipPainterNode::computeTargetPose() {
    geometry_msgs::PoseStamped target;
    target.header.frame_id = params_.frame_id;
    target.header.stamp = ros::Time::now();
    
    if (flight_state_.current_layer_idx >= path_layers_.size()) {
        target.pose = flight_state_.home_pose.pose;
        return target;
    }
    
    PathPlanner::Waypoint wp = getCurrentWaypoint();
    
    // 位置
    target.pose.position.x = wp.position.x();
    target.pose.position.y = wp.position.y();
    target.pose.position.z = wp.position.z();
    
    // 计算喷涂姿态
    Eigen::Vector3d current_pos(
        flight_state_.current_pose.pose.position.x,
        flight_state_.current_pose.pose.position.y,
        flight_state_.current_pose.pose.position.z
    );
    
    Eigen::Quaterniond q = computeSprayOrientation(wp, current_pos);
    
    // 平滑过渡
    if (flight_state_.has_last_state) {
        q = smoothOrientation(q, flight_state_.last_orientation, 0.1);
        flight_state_.last_orientation = q;
    } else {
        flight_state_.last_orientation = q;
        flight_state_.has_last_state = true;
    }
    
    target.pose.orientation.w = q.w();
    target.pose.orientation.x = q.x();
    target.pose.orientation.y = q.y();
    target.pose.orientation.z = q.z();
    
    return target;
}

mavros_msgs::PositionTarget ShipPainterNode::computePositionTarget() {
    mavros_msgs::PositionTarget target;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = params_.frame_id;
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    
    PathPlanner::Waypoint wp = getCurrentWaypoint();
    
    // 设置位置
    target.position.x = wp.position.x();
    target.position.y = wp.position.y();
    target.position.z = wp.position.z();
    
    // 计算速度（用于平滑移动）
    Eigen::Vector3d target_pos(wp.position.x(), wp.position.y(), wp.position.z());
    Eigen::Vector3d current_pos(
        flight_state_.current_pose.pose.position.x,
        flight_state_.current_pose.pose.position.y,
        flight_state_.current_pose.pose.position.z
    );
    
    Eigen::Vector3d direction = target_pos - current_pos;
    double distance = direction.norm();
    
    if (distance > 0.1) {
        direction.normalize();
        double speed = (distance > 1.0) ? params_.flight_speed : params_.approach_speed;
        target.velocity.x = direction.x() * speed;
        target.velocity.y = direction.y() * speed;
        target.velocity.z = direction.z() * speed;
    } else {
        target.velocity.x = 0;
        target.velocity.y = 0;
        target.velocity.z = 0;
    }
    
    // 计算姿态
    // 1. 基础喷涂姿态（偏航角和俯仰角）
    Eigen::Vector3d spray_direction = -wp.surface_normal;
    spray_direction.normalize();
    
    double yaw = std::atan2(spray_direction.y(), spray_direction.x());
    double pitch = std::asin(-spray_direction.z());
    
    // 2. 如果需要俯仰调整（非垂直表面）
    if (wp.needs_pitch_adjustment) {
        pitch = std::max(-params_.max_pitch_adjustment, 
                        std::min(params_.max_pitch_adjustment, pitch));
    }
    
    // 3. 计算横向移动所需的roll角
    double roll = 0.0;
    if (flight_state_.current_waypoint_idx > 0 && distance > 0.1) {
        roll = computeRollForLateralMovement(current_pos, target_pos, yaw);
    }
    
    // 设置姿态（用欧拉角）
    target.yaw = yaw;
    target.yaw_rate = 0;
    
    // 创建四元数（用于更精确的控制）
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    
    // 设置type_mask（控制哪些通道）
    target.type_mask = 
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ;
    
    return target;
}

Eigen::Quaterniond ShipPainterNode::computeSprayOrientation(
    const PathPlanner::Waypoint& waypoint,
    const Eigen::Vector3d& current_pos) {
    
    // 喷头需要指向表面（-surface_normal方向）
    Eigen::Vector3d spray_direction = -waypoint.surface_normal;
    spray_direction.normalize();
    
    // 计算偏航角（水平旋转）
    double yaw = std::atan2(spray_direction.y(), spray_direction.x());
    
    // 计算俯仰角
    double pitch = std::asin(-spray_direction.z());
    
    // 如果需要俯仰调整，限制俯仰角
    if (waypoint.needs_pitch_adjustment) {
        pitch = std::max(-params_.max_pitch_adjustment, 
                        std::min(params_.max_pitch_adjustment, pitch));
    }
    
    // 计算roll角（用于横向移动）
    double roll = 0.0;
    if (flight_state_.current_waypoint_idx > 0) {
        Eigen::Vector3d movement = waypoint.position - current_pos;
        movement.z() = 0;  // 只考虑水平移动
        
        if (movement.norm() > 0.1) {
            // 将移动向量转换到无人机坐标系
            Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).matrix();
            Eigen::Vector3d movement_body = R_yaw.transpose() * movement;
            
            // 根据Y方向移动计算roll
            roll = std::atan2(movement_body.y(), movement.norm()) * params_.roll_gain;
            roll = std::max(-params_.max_roll, std::min(params_.max_roll, roll));
        }
    }
    
    // 创建四元数
    Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    
    return Eigen::Quaterniond(yaw_angle * pitch_angle * roll_angle);
}

double ShipPainterNode::computeRollForLateralMovement(
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    double current_yaw) {
    
    // 计算移动向量
    Eigen::Vector3d movement = to - from;
    movement.z() = 0;  // 只考虑水平移动
    
    if (movement.norm() < 0.01) return 0.0;
    
    // 将移动向量转换到无人机坐标系
    Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(current_yaw, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Vector3d movement_body = R_yaw.transpose() * movement;
    
    // 计算所需的roll角
    // 正roll产生右移，负roll产生左移
    double desired_lateral_speed = movement_body.y() / movement.norm() * params_.flight_speed;
    double roll = std::atan(desired_lateral_speed / 9.81) * params_.roll_gain;
    
    // 限制roll角
    roll = std::max(-params_.max_roll, std::min(params_.max_roll, roll));
    
    return roll;
}

bool ShipPainterNode::isWaypointReached() {
    PathPlanner::Waypoint wp = getCurrentWaypoint();
    
    // 计算位置误差
    geometry_msgs::Point target_pos;
    target_pos.x = wp.position.x();
    target_pos.y = wp.position.y();
    target_pos.z = wp.position.z();
    
    double pos_error = calculateDistance(flight_state_.current_pose.pose.position, target_pos);
    
    // 对于喷涂任务，主要关注位置精度
    return pos_error < flight_state_.position_tolerance;
}

void ShipPainterNode::advanceToNextWaypoint() {
    const auto& current_layer = path_layers_[flight_state_.current_layer_idx];
    
    flight_state_.current_waypoint_idx++;
    flight_state_.waypoint_reached = false;
    flight_state_.total_waypoints_reached++;
    
    if (flight_state_.current_waypoint_idx >= current_layer.waypoints.size()) {
        ROS_INFO("Completed layer %zu/%zu",
                 flight_state_.current_layer_idx + 1, path_layers_.size());
        
        flight_state_.current_layer_idx++;
        flight_state_.current_waypoint_idx = 0;
        
        if (flight_state_.current_layer_idx >= path_layers_.size()) {
            flight_state_.mission_complete = true;
            ROS_INFO("Mission complete! Total waypoints: %zu",
                     flight_state_.total_waypoints_reached);
        }
    }
}

PathPlanner::Waypoint ShipPainterNode::getCurrentWaypoint() {
    if (flight_state_.current_layer_idx < path_layers_.size()) {
        const auto& layer = path_layers_[flight_state_.current_layer_idx];
        if (flight_state_.current_waypoint_idx < layer.waypoints.size()) {
            return layer.waypoints[flight_state_.current_waypoint_idx];
        }
    }
    
    // 返回home位置作为默认
    PathPlanner::Waypoint home_wp;
    home_wp.position = Eigen::Vector3d(
        flight_state_.home_pose.pose.position.x,
        flight_state_.home_pose.pose.position.y,
        flight_state_.home_pose.pose.position.z);
    home_wp.surface_normal = Eigen::Vector3d(0, 0, -1);  // 向上
    home_wp.approach_dir = Eigen::Vector3d(0, 0, 1);     // 向下接近
    return home_wp;
}

PathPlanner::Waypoint ShipPainterNode::getInterpolatedWaypoint(double alpha) {
    // 获取当前和下一个航点，进行插值
    PathPlanner::Waypoint current = getCurrentWaypoint();
    
    if (flight_state_.current_waypoint_idx + 1 < 
        path_layers_[flight_state_.current_layer_idx].waypoints.size()) {
        
        PathPlanner::Waypoint next = 
            path_layers_[flight_state_.current_layer_idx]
            .waypoints[flight_state_.current_waypoint_idx + 1];
        
        PathPlanner::Waypoint interpolated;
        interpolated.position = (1.0 - alpha) * current.position + alpha * next.position;
        interpolated.surface_normal = ((1.0 - alpha) * current.surface_normal + 
                                       alpha * next.surface_normal).normalized();
        interpolated.approach_dir = -interpolated.surface_normal;
        interpolated.needs_pitch_adjustment = current.needs_pitch_adjustment || 
                                              next.needs_pitch_adjustment;
        
        return interpolated;
    }
    
    return current;
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

bool ShipPainterNode::executeSprayMission() {
    if (!flight_state_.path_ready || path_layers_.empty()) {
        ROS_ERROR("No path to follow");
        return false;
    }
    
    ROS_INFO("Starting spray painting mission...");
    ROS_INFO("Control mode: %s", params_.use_position_control ? "Position" : "Position+Attitude");
    
    control_timer_.start();
    
    ros::Rate rate(20);
    while (ros::ok() && !flight_state_.mission_complete) {
        if (!flight_state_.mavros_state.connected || !flight_state_.mavros_state.armed) {
            ROS_ERROR("Lost connection or disarmed");
            control_timer_.stop();
            return false;
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    control_timer_.stop();
    return flight_state_.mission_complete;
}

// 可视化函数
void ShipPainterNode::visualizationTimerCallback(const ros::TimerEvent& event) {
    publishPathVisualization();
    publishSprayVisualization();
    publishModelVisualization();
    publishStatusVisualization();
    publishPointCloudVisualization();//点云可视化
    // publishNormalVisualization();//法向量可视化
    publishAlphaShapeVisualization(); //as轮廓可视化
    publishOffsetContourVisualization();//偏移轮廓可视化
}
//按层划分路径线
void ShipPainterNode::publishPathVisualization() {
    if (!flight_state_.path_ready) return;
    
    visualization_msgs::MarkerArray marker_array;
    
    for (size_t layer_idx = 0; layer_idx < path_layers_.size(); ++layer_idx) {
        const auto& layer = path_layers_[layer_idx];
        
        // 路径线
        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = ros::Time::now();
        path_marker.ns = "layer_path";
        path_marker.id = layer_idx;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.scale.x = 0.02;
        
        // 颜色编码
        if (layer_idx < flight_state_.current_layer_idx) {
            // 已完成层 - 灰色
            path_marker.color.r = 0.3;
            path_marker.color.g = 0.3;
            path_marker.color.b = 0.3;
            path_marker.color.a = 0.5;
        } else if (layer_idx == flight_state_.current_layer_idx) {
            // 当前层 - 绿色
            path_marker.color.r = 0.0;
            path_marker.color.g = 1.0;
            path_marker.color.b = 0.0;
            path_marker.color.a = 1.0;
        } else {
            // 未完成层 - 橙色
            path_marker.color.r = 1.0;
            path_marker.color.g = 0.5;
            path_marker.color.b = 0.0;
            path_marker.color.a = 0.7;
        }
        
        for (const auto& wp : layer.waypoints) {
            geometry_msgs::Point p;
            p.x = wp.position.x();
            p.y = wp.position.y();
            p.z = wp.position.z();
            path_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(path_marker);
        
        // 航点球
        visualization_msgs::Marker wp_marker;
        wp_marker.header = path_marker.header;
        wp_marker.ns = "waypoints";
        wp_marker.id = layer_idx + 1000;
        wp_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        wp_marker.action = visualization_msgs::Marker::ADD;
        wp_marker.scale.x = 0.05;
        wp_marker.scale.y = 0.05;
        wp_marker.scale.z = 0.05;
        wp_marker.color = path_marker.color;
        wp_marker.points = path_marker.points;
        
        marker_array.markers.push_back(wp_marker);
        
        // 法向量箭头（显示喷涂方向）
        if (layer_idx == flight_state_.current_layer_idx) {
            visualization_msgs::Marker normal_marker;
            normal_marker.header = path_marker.header;
            normal_marker.ns = "normals";
            normal_marker.id = layer_idx + 2000;
            normal_marker.type = visualization_msgs::Marker::ARROW;
            normal_marker.action = visualization_msgs::Marker::ADD;
            normal_marker.scale.x = 0.01;  // 箭杆直径
            normal_marker.scale.y = 0.02;  // 箭头直径
            normal_marker.scale.z = 0.02;  // 箭头长度
            normal_marker.color.r = 0.0;
            normal_marker.color.g = 0.0;
            normal_marker.color.b = 1.0;
            normal_marker.color.a = 0.8;
            
            if (flight_state_.current_waypoint_idx < layer.waypoints.size()) {
                const auto& wp = layer.waypoints[flight_state_.current_waypoint_idx];
                
                geometry_msgs::Point start, end;
                start.x = wp.position.x();
                start.y = wp.position.y();
                start.z = wp.position.z();
                
                Eigen::Vector3d end_pos = wp.position - wp.surface_normal * 0.3;
                end.x = end_pos.x();
                end.y = end_pos.y();
                end.z = end_pos.z();
                
                normal_marker.points.push_back(start);
                normal_marker.points.push_back(end);
                
                marker_array.markers.push_back(normal_marker);
            }
        }
    }
    
    path_vis_pub_.publish(marker_array);
}

void ShipPainterNode::publishSprayVisualization() {
    if (!params_.enable_spray_visualization || !flight_state_.path_ready) return;
    
    visualization_msgs::MarkerArray marker_array;
    
    // 当前喷涂区域
    if (flight_state_.current_layer_idx < path_layers_.size()) {
        const auto& layer = path_layers_[flight_state_.current_layer_idx];
        if (flight_state_.current_waypoint_idx < layer.waypoints.size()) {
            const auto& wp = layer.waypoints[flight_state_.current_waypoint_idx];
            
            // 喷涂圆锥
            visualization_msgs::Marker spray_marker;
            spray_marker.header.frame_id = "map";
            spray_marker.header.stamp = ros::Time::now();
            spray_marker.ns = "spray_cone";
            spray_marker.id = 0;
            spray_marker.type = visualization_msgs::Marker::CYLINDER;
            spray_marker.action = visualization_msgs::Marker::ADD;
            
            // 位置在航点和表面之间
            Eigen::Vector3d spray_center = wp.position - wp.surface_normal * params_.spray_distance * 0.5;
            spray_marker.pose.position.x = spray_center.x();
            spray_marker.pose.position.y = spray_center.y();
            spray_marker.pose.position.z = spray_center.z();
            
            // 朝向表面
            Eigen::Vector3d spray_dir = -wp.surface_normal;
            Eigen::Quaterniond q = computeSprayOrientation(wp, wp.position);
            spray_marker.pose.orientation.w = q.w();
            spray_marker.pose.orientation.x = q.x();
            spray_marker.pose.orientation.y = q.y();
            spray_marker.pose.orientation.z = q.z();
            
            spray_marker.scale.x = params_.spray_radius * 2;
            spray_marker.scale.y = params_.spray_radius * 2;
            spray_marker.scale.z = params_.spray_distance;
            
            spray_marker.color.r = 0.0;
            spray_marker.color.g = 0.5;
            spray_marker.color.b = 1.0;
            spray_marker.color.a = 0.3;
            
            marker_array.markers.push_back(spray_marker);
            
            // 喷涂圆面（在表面上）
            visualization_msgs::Marker spray_circle;
            spray_circle.header = spray_marker.header;
            spray_circle.ns = "spray_circle";
            spray_circle.id = 1;
            spray_circle.type = visualization_msgs::Marker::CYLINDER;
            spray_circle.action = visualization_msgs::Marker::ADD;
            
            Eigen::Vector3d surface_pos = wp.position - wp.surface_normal * params_.spray_distance;
            spray_circle.pose.position.x = surface_pos.x();
            spray_circle.pose.position.y = surface_pos.y();
            spray_circle.pose.position.z = surface_pos.z();
            spray_circle.pose.orientation = spray_marker.pose.orientation;
            
            spray_circle.scale.x = params_.spray_radius * 2;
            spray_circle.scale.y = params_.spray_radius * 2;
            spray_circle.scale.z = 0.01;  // 很薄的圆盘
            
            spray_circle.color.r = 0.0;
            spray_circle.color.g = 1.0;
            spray_circle.color.b = 0.0;
            spray_circle.color.a = 0.5;
            
            marker_array.markers.push_back(spray_circle);
        }
    }
    
    spray_vis_pub_.publish(marker_array);
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
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            contour_marker.points.push_back(p);
        }
        
        // 确保轮廓闭合
        if (contour.size() > 2) {
            geometry_msgs::Point first_pt;
            first_pt.x = contour[0].x();
            first_pt.y = contour[0].y();
            first_pt.z = contour[0].z();
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
    
    ROS_INFO_THROTTLE(5, "Publishing %zu offset contours for visualization", offset_contours.size());
    
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
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z() + 0.01; // 稍微抬高一点，避免与其他线条重叠
            contour_marker.points.push_back(p);
        }
        
        // 确保轮廓闭合
        if (contour.size() > 2) {
            geometry_msgs::Point first_pt;
            first_pt.x = contour[0].x();
            first_pt.y = contour[0].y();
            first_pt.z = contour[0].z() + 0.01;
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
        size_t step = std::max(1UL, contour.size() / 50);  // 最多显示50个点
        for (size_t j = 0; j < contour.size(); j += step) {
            geometry_msgs::Point p;
            p.x = contour[j].x();
            p.y = contour[j].y();
            p.z = contour[j].z() + 0.02; // 比线条再高一点
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
    static int debug_count = 0;
    if (++debug_count % 100 == 0) {
        ROS_INFO("Published offset contour visualization: %zu layers, %zu markers total",
                 offset_contours.size(), marker_array.markers.size());
    }
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
    status_marker.pose.position.z = params_.model_z;
    status_marker.pose.orientation.w = 1.0;
    
    status_marker.scale.z = 0.3;
    
    status_marker.color.r = 1.0;
    status_marker.color.g = 1.0;
    status_marker.color.b = 1.0;
    status_marker.color.a = 1.0;
    
    std::stringstream ss;
    if (flight_state_.mission_complete) {
        ss << "MISSION COMPLETE\n";
        ss << "Total waypoints: " << flight_state_.total_waypoints_reached;
    } else if (flight_state_.path_ready) {
        ss << "Layer: " << (flight_state_.current_layer_idx + 1) 
           << "/" << path_layers_.size() << "\n";
        
        if (flight_state_.current_layer_idx < path_layers_.size()) {
            const auto& layer = path_layers_[flight_state_.current_layer_idx];
            ss << "Waypoint: " << (flight_state_.current_waypoint_idx + 1) 
               << "/" << layer.waypoints.size() << "\n";
            ss << "Height: " << std::fixed << std::setprecision(3) 
               << layer.z_center << " m\n";
        }
        
        ss << "Progress: " << flight_state_.total_waypoints_reached << " waypoints";
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
        p.x = point.x + params_.model_x;  // 应用模型位置偏移
        p.y = point.y + params_.model_y;
        p.z = point.z + params_.model_z;
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
            p.x = point.x + params_.model_x;
            p.y = point.y + params_.model_y;
            p.z = point.z + params_.model_z;
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
    static int debug_counter = 0;
    if (debug_counter % 50 == 0) {  // 每5秒打印一次（假设10Hz调用频率）
        ROS_INFO_THROTTLE(5, "Publishing pointcloud visualization: %zu processed points,%zu original points",
                         processed_cloud->size(), original_cloud ? original_cloud->size() : 0);
    }
    debug_counter++;
}

//法向量可视化函数
void ShipPainterNode::publishNormalVisualization() {
    
    visualization_msgs::MarkerArray marker_array;
    
    // 1. 显示点云法向量（绿色箭头）
    pcl::PointCloud<pcl::PointNormal>::Ptr processed_cloud = planner_.getProcessedCloud();
    if (processed_cloud && !processed_cloud->empty()) {
        visualization_msgs::Marker pointcloud_normals;
        pointcloud_normals.header.frame_id = params_.frame_id;
        pointcloud_normals.header.stamp = ros::Time::now();
        pointcloud_normals.ns = "pointcloud_normals";
        pointcloud_normals.id = 0;
        pointcloud_normals.type = visualization_msgs::Marker::LINE_LIST;
        pointcloud_normals.action = visualization_msgs::Marker::ADD;
        
        pointcloud_normals.scale.x = 0.005;  // 箭杆直径
        // pointcloud_normals.scale.y = 0.1;   // 箭头直径
        // pointcloud_normals.scale.z = 0.1;   // 箭头长度
        
        pointcloud_normals.color.r = 0.0;
        pointcloud_normals.color.g = 1.0;    // 绿色
        pointcloud_normals.color.b = 0.0;
        pointcloud_normals.color.a = 0.6;
        
        // 采样显示法向量（避免太密集）
        size_t step = std::max(1UL, processed_cloud->size() / 1000);  // 最多显示1000个
        for (size_t i = 0; i < processed_cloud->size(); i += step) {
            const auto& point = processed_cloud->points[i];
            
            if (!pcl::isFinite(point)) continue;
            
            // 法向量长度
            double normal_length = 0.5;  // 5cm
            
            geometry_msgs::Point start, end;
            start.x = point.x + params_.model_x;
            start.y = point.y + params_.model_y;
            start.z = point.z + params_.model_z;
            
            end.x = start.x + point.normal_x * normal_length;
            end.y = start.y + point.normal_y * normal_length;
            end.z = start.z + point.normal_z * normal_length;
            
            pointcloud_normals.points.push_back(start);
            pointcloud_normals.points.push_back(end);
        }
        
        if (!pointcloud_normals.points.empty()) {
            marker_array.markers.push_back(pointcloud_normals);
        }
        // ROS_INFO("Publishing %zu normal arrows", pointcloud_normals.points.size()/2);
    }
    
    // 2. 显示路径点法向量（蓝色箭头）
    if (flight_state_.path_ready && !path_layers_.empty()) {
        visualization_msgs::Marker waypoint_normals;
        waypoint_normals.header.frame_id = params_.frame_id;
        waypoint_normals.header.stamp = ros::Time::now();
        waypoint_normals.ns = "waypoint_normals";
        waypoint_normals.id = 1;
        waypoint_normals.type = visualization_msgs::Marker::LINE_LIST;
        waypoint_normals.action = visualization_msgs::Marker::ADD;
        
        waypoint_normals.scale.x = 0.008;   // 稍粗一点
        // waypoint_normals.scale.y = 0.15;
        // waypoint_normals.scale.z = 0.15;
        
        waypoint_normals.color.r = 0.0;
        waypoint_normals.color.g = 0.0;
        waypoint_normals.color.b = 1.0;    // 蓝色
        waypoint_normals.color.a = 0.8;
        
        // 显示所有层的航点法向量
        for (const auto& layer : path_layers_) {
            for (const auto& wp : layer.waypoints) {
                double normal_length = 0.8;  // 8cm，比点云法向量长一些
                
                geometry_msgs::Point start, end;
                start.x = wp.position.x();
                start.y = wp.position.y();
                start.z = wp.position.z();
                
                end.x = start.x + wp.surface_normal.x() * normal_length;
                end.y = start.y + wp.surface_normal.y() * normal_length;
                end.z = start.z + wp.surface_normal.z() * normal_length;
                
                waypoint_normals.points.push_back(start);
                waypoint_normals.points.push_back(end);
            }
        }
        
        if (!waypoint_normals.points.empty()) {
            marker_array.markers.push_back(waypoint_normals);
        }
    }
    
    normal_vis_pub_.publish(marker_array);
    
    // 调试信息
    static int debug_counter = 0;
    if (debug_counter % 100 == 0) {
        ROS_INFO_THROTTLE(10, "Publishing normal vectors visualization");
    }
    debug_counter++;
}

// 主要任务执行函数保持不变...
bool ShipPainterNode::waitForConnection() {
    ROS_INFO("Waiting for MAVROS connection...");
    ros::Rate rate(20);
    for (int i = 0; i < 100; ++i) {
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
    
    // 2. 等待位置数据
    ROS_INFO("Waiting for position data...");
    ros::Rate rate(20);
    while (ros::ok() && !flight_state_.home_set) {
        ros::spinOnce();
        rate.sleep();
    }
    
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
    
    // 8. 执行喷涂任务
    bool mission_success = executeSprayMission();
    
    // 9. 降落
    if (mission_success) {
        ROS_INFO("Mission successful! Landing...");
    } else {
        ROS_WARN("Mission interrupted. Landing...");
    }
    
    setMode("AUTO.LAND");
    ros::Duration(10.0).sleep();
    
    // 10. 上锁
    disarm();
    
    ROS_INFO("Mission ended. Total waypoints reached: %zu", 
            flight_state_.total_waypoints_reached);
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