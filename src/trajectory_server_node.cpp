#include <ros/ros.h>
#include "ship_painter/Bspline.h"
#include "ship_painter/BsplineLayer.h"
#include "ship_painter/bspline.h"
#include <mavros_msgs/PositionTarget.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseStamped.h>  // 用于获取姿态
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


class TrajectoryServer {
public:
    TrajectoryServer() : nh_("~") {
    // 获取mavros命名空间参数
    std::string mavros_ns;
    nh_.param<std::string>("mavros_ns", mavros_ns, "/mavros");
    
    // 订阅B样条轨迹
    bspline_sub_ = nh_.subscribe("/planning/bspline_layers", 1,
        &TrajectoryServer::bsplineCallback, this);
    
    // 使用带命名空间的话题
    setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
        mavros_ns + "/setpoint_raw/local", 10);

    trajectory_vis_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/visualization/current_trajectory", 10);  // B样条颜色
    
    // 打印调试信息
    ROS_INFO("Trajectory server publishing to: %s/setpoint_raw/local", mavros_ns.c_str());
        
        // 控制定时器（100Hz）
        control_timer_ = nh_.createTimer(
            ros::Duration(0.005),
            &TrajectoryServer::controlLoop, this);
        
        current_layer_idx_ = 0;
        trajectory_active_ = false;

    //加载参数
    loadParameters();
    
    //深度相机订阅
    if (enable_depth_correction_) {
        depth_sub_ = nh_.subscribe("/camera/aligned_depth_to_color/image_raw", 
                                   1, &TrajectoryServer::depthCallback, this);
        ROS_INFO("Depth correction enabled. Subscribing to depth image...");
    } else {
        ROS_INFO("Depth correction disabled.");
    }
    
    //姿态订阅（获取yaw角
    pose_sub_ = nh_.subscribe(mavros_ns + "/local_position/pose", 
                              10, &TrajectoryServer::poseCallback, this);
    
    // 打印配置信息
    if (enable_depth_correction_) {
        ROS_INFO("=== Depth Correction Configuration ===");
        ROS_INFO("  Target distance: %.2f m", target_distance_);
        ROS_INFO("  Kp: %.2f, max correction: %.2f m", distance_kp_, max_correction_);
        ROS_INFO("  Camera offset: [%.3f, %.3f, %.3f] m (body frame)", 
                 camera_offset_x_, camera_offset_y_, camera_offset_z_);
        ROS_INFO("  Depth range: [%.2f, %.2f] m", depth_min_range_, depth_max_range_);
    }
}
    }
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber bspline_sub_;
    ros::Publisher setpoint_pub_;
    ros::Timer control_timer_;

    ros::Publisher trajectory_vis_pub_; 
    
    std::vector<ship_painter::BSpline> layers_;
    std::vector<ship_painter::BSpline> transitions_;
    
    size_t current_layer_idx_;
    bool trajectory_active_;
    ros::Time trajectory_start_time_;

    //深度控制相关 
    // 订阅器
    ros::Subscriber depth_sub_;           // 深度图像订阅
    ros::Subscriber pose_sub_;            // 姿态订阅（获取yaw）
    
    // 深度状态
    struct DepthState {
        float distance;                   // 当前距离（米）
        bool valid;                       // 数据有效性
        ros::Time last_update;            // 最后更新时间
        float last_distance;              // 上次距离（用于突变检测）
        
        DepthState() : distance(0.0), valid(false), last_distance(0.0) {}
    } depth_state_;
    
    // 当前姿态
    geometry_msgs::PoseStamped current_pose_;
    bool pose_received_ = false;
    
    // 控制参数（从launch文件读取）
    bool enable_depth_correction_;        // 功能开关
    float target_distance_;               // 目标距离（米）
    float distance_kp_;                   // 比例增益
    float distance_ki_;                   // 积分增益（暂不用）
    float distance_kd_;                   // 微分增益（暂不用）
    float max_correction_;                // 最大修正量（米）
    
    // 相机偏移参数（机体坐标系）
    float camera_offset_x_;               // 前后偏移（正=前）
    float camera_offset_y_;               // 左右偏移（正=右）
    float camera_offset_z_;               // 上下偏移（正=上）
    
    // 深度数据有效性检查参数
    float depth_timeout_;                 // 超时时间（秒）
    float depth_min_range_;               // 最小有效距离（米）
    float depth_max_range_;               // 最大有效距离（米）
    float depth_jump_threshold_;          // 突变阈值（米）
    
    bool debug_output_;                   // 调试输出开关
    
    //方法声明
    void depthCallback(const sensor_msgs::ImageConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool checkDepthValid();
    double getCurrentYaw();
    void loadParameters();
    //

    void bsplineCallback(const ship_painter::BsplineLayer::ConstPtr& msg);
    void controlLoop(const ros::TimerEvent& event);
};

void TrajectoryServer::bsplineCallback(
    const ship_painter::BsplineLayer::ConstPtr& msg) {
    
    layers_.clear();
    
    for (const auto& layer_msg : msg->layers) {
        ship_painter::BSpline bspline;
        
        std::vector<Eigen::Vector3d> control_points;
        std::vector<Eigen::Vector3d> normals;
        
        for (const auto& pt : layer_msg.pos_pts) {
            control_points.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
        }
        
        for (const auto& n : layer_msg.normals) {
            normals.push_back(Eigen::Vector3d(n.x, n.y, n.z));
        }
        
        // 直接设置参数，不重新拟合
        bspline.setControlPoints(control_points);
        bspline.setKnots(layer_msg.knots);
        bspline.setNormals(normals);
        bspline.setDegree(layer_msg.order);
        bspline.setTotalTime(layer_msg.duration);
        
        layers_.push_back(bspline);
    }
    
    // 启动轨迹跟踪
    trajectory_active_ = true;
    trajectory_start_time_ = ros::Time::now();  //立即开始，不用消息中的start_time
    current_layer_idx_ = 0;
    
    ROS_INFO("Trajectory server: Loaded %zu layers", layers_.size());

    if (!layers_.empty()) {
        const auto& first_traj = layers_[0];
        Eigen::Vector3d p = first_traj.getPosition(0.0);
        Eigen::Vector3d v = first_traj.getVelocity(0.0);
        Eigen::Vector3d n = first_traj.getNormal(0.0);
        
        mavros_msgs::PositionTarget target;
        target.header.stamp = ros::Time::now();
        target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        
        target.position.x = p.x();
        target.position.y = p.y();
        target.position.z = p.z();
        
        target.velocity.x = v.x();
        target.velocity.y = v.y();
        target.velocity.z = v.z();
        
        target.yaw = std::atan2(-n.y(), -n.x());
        target.yaw_rate = 0.0;
        
        target.type_mask = 
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ;
        
        setpoint_pub_.publish(target);
        ROS_INFO("Published first setpoint immediately");
    }
}

void TrajectoryServer::controlLoop(const ros::TimerEvent& event) {
    if (!trajectory_active_ || layers_.empty()) 
        {
        //每5秒打印一次状态
        ROS_WARN_THROTTLE(5, "Trajectory server waiting (active=%d, layers=%zu)", 
                         trajectory_active_, layers_.size());
        return;
    }
    
    // 计算当前时间
    double t = (ros::Time::now() - trajectory_start_time_).toSec();
    
    // 检查是否需要切换层
    if (current_layer_idx_ < layers_.size()) {
        const auto& current_traj = layers_[current_layer_idx_];
        
        if (t > current_traj.getTotalTime()) {
            // 切换到下一层
            current_layer_idx_++;
            trajectory_start_time_ = ros::Time::now();
            t = 0.0;
            
            if (current_layer_idx_ >= layers_.size()) {
                ROS_INFO_ONCE("All layers completed! Hovering at final position...");
    
                // 获取最后一层的最后位置
                const auto& last_traj = layers_.back();
                double last_t = last_traj.getTotalTime();
                
                Eigen::Vector3d p = last_traj.getPosition(last_t);
                Eigen::Vector3d v = Eigen::Vector3d::Zero();  // hover速度为0
                Eigen::Vector3d n = last_traj.getNormal(last_t);
                double yaw = std::atan2(-n.y(), -n.x());
            
                // 发布hover指令
                mavros_msgs::PositionTarget target;
                target.header.stamp = ros::Time::now();
                target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                
                target.position.x = p.x();
                target.position.y = p.y();
                target.position.z = p.z();
                
                target.velocity.x = 0.0;
                target.velocity.y = 0.0;
                target.velocity.z = 0.0;
                
                target.yaw = yaw;
                target.yaw_rate = 0.0;
                
                target.type_mask = 
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ;
                
                setpoint_pub_.publish(target);
                
                ROS_INFO_THROTTLE(2.0, "Hovering at final position: [%.2f, %.2f, %.2f]",
                                p.x(), p.y(), p.z());
                return;  // 继续hover，不要设置trajectory_active_ = false
            }
        }
        
        // 查询当前状态
        Eigen::Vector3d p = current_traj.getPosition(t);
        Eigen::Vector3d v = current_traj.getVelocity(t);
        Eigen::Vector3d n = current_traj.getNormal(t);

         //深度修正逻辑
        Eigen::Vector3d p_corrected = p;  // 默认使用原始位置
        
        if (enable_depth_correction_ && checkDepthValid()) {
            // 1. 获取当前yaw角
            double yaw = getCurrentYaw();
            
            // 2. 考虑相机偏移的实际距离（沿机体X轴）
            float actual_distance_from_fc = depth_state_.distance + camera_offset_x_;
            
            // 3. 计算距离误差
            float error = actual_distance_from_fc - target_distance_;
            
            // 4. PID计算修正量（机体坐标系，沿前方）
            float correction_body_x = distance_kp_ * error;
            
            // 5. 限幅保护
            if (std::abs(correction_body_x) > max_correction_) {
                correction_body_x = (correction_body_x > 0) ? 
                                    max_correction_ : -max_correction_;
                if (debug_output_) {
                    ROS_WARN_THROTTLE(1, "Correction saturated: %.3f m", correction_body_x);
                }
            }
            
            // 6. 转换到世界坐标系
            Eigen::Vector3d correction_world;
            correction_world.x() = correction_body_x * cos(yaw);
            correction_world.y() = correction_body_x * sin(yaw);
            correction_world.z() = 0.0;  // 不修正高度
            
            // 7. 应用修正
            p_corrected = p + correction_world;
            
            // 8. 调试输出
            if (debug_output_) {
                ROS_INFO_THROTTLE(0.5, 
                    "Depth Correction | "
                    "d_cam=%.3f, d_fc=%.3f, error=%.3f | "
                    "corr_body=%.3f, corr_world=[%.3f,%.3f] | "
                    "yaw=%.1f°",
                    depth_state_.distance, actual_distance_from_fc, error,
                    correction_body_x, 
                    correction_world.x(), correction_world.y(),
                    yaw * 180.0 / M_PI);
            }
        } else if (enable_depth_correction_ && !checkDepthValid()) {
            // 深度数据无效，使用GPS定位
            if (debug_output_) {
                ROS_WARN_THROTTLE(2, "Depth invalid, using GPS-only mode");
            }
        }
        
        // 计算yaw（保持机头朝向表面）
        Eigen::Vector3d spray_dir = -n;
        double yaw = std::atan2(spray_dir.y(), spray_dir.x());
        
        // 发布修正后的目标
        mavros_msgs::PositionTarget target;
        target.header.stamp = ros::Time::now();
        target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        
        target.position.x = p_corrected.x();  // 使用修正后的位置
        target.position.y = p_corrected.y();
        target.position.z = p_corrected.z();
        
        target.velocity.x = v.x();
        target.velocity.y = v.y();
        target.velocity.z = v.z();
        
        target.yaw = yaw;
        target.yaw_rate = 0.0;
        
        target.type_mask = 
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ;
        
        setpoint_pub_.publish(target);

        //打印当前状态
        ROS_INFO_THROTTLE(1.0, "Layer %zu/%zu, t=%.2f/%.2f, pos=[%.2f, %.2f, %.2f]",
                        current_layer_idx_ + 1, layers_.size(),
                        t, current_traj.getTotalTime(),
                        p.x(), p.y(), p.z());

        //发布可视化
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "executed_trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;  // 线宽
        
        // 已走过的部分：绿色
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        // 从轨迹起点到当前点采样
        double total_time = current_traj.getTotalTime();
        for (double sample_t = 0; sample_t <= t && sample_t <= total_time; sample_t += 0.05) {
            Eigen::Vector3d pos = current_traj.getPosition(sample_t);
            geometry_msgs::Point p;
            p.x = pos.x();
            p.y = pos.y();
            p.z = pos.z();
            marker.points.push_back(p);
        }
        
        trajectory_vis_pub_.publish(marker);
        
        setpoint_pub_.publish(target);
    }
}

// 加载参数
void TrajectoryServer::loadParameters() {
    // 功能开关
    nh_.param("enable_depth_correction", enable_depth_correction_, true);
    
    // 目标距离
    nh_.param("target_distance", target_distance_, 0.6f);
    
    // PID参数
    nh_.param("distance_kp", distance_kp_, 0.5f);
    nh_.param("distance_ki", distance_ki_, 0.0f);
    nh_.param("distance_kd", distance_kd_, 0.0f);
    
    // 修正限制
    nh_.param("max_correction", max_correction_, 0.2f);
    
    // 相机偏移（机体坐标系）
    nh_.param("camera_offset_x", camera_offset_x_, 0.0f);
    nh_.param("camera_offset_y", camera_offset_y_, 0.0f);
    nh_.param("camera_offset_z", camera_offset_z_, 0.0f);
    
    // 深度数据检查参数
    nh_.param("depth_timeout", depth_timeout_, 0.5f);
    nh_.param("depth_min_range", depth_min_range_, 0.3f);
    nh_.param("depth_max_range", depth_max_range_, 2.0f);
    nh_.param("depth_jump_threshold", depth_jump_threshold_, 0.5f);
    
    // 调试开关
    nh_.param("debug_output", debug_output_, false);
}

// 深度图像回调
void TrajectoryServer::depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // 转换为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
            msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::Mat depth_image = cv_ptr->image;
        
        // 获取图像中心坐标
        int center_x = depth_image.cols / 2;
        int center_y = depth_image.rows / 2;
        
        // 提取中心区域（20x20像素）的平均深度
        int region_size = 10;
        float sum = 0.0;
        int valid_count = 0;
        
        for (int dy = -region_size; dy <= region_size; dy++) {
            for (int dx = -region_size; dx <= region_size; dx++) {
                int x = center_x + dx;
                int y = center_y + dy;
                
                if (x >= 0 && x < depth_image.cols && 
                    y >= 0 && y < depth_image.rows) {
                    uint16_t depth_mm = depth_image.at<uint16_t>(y, x);
                    if (depth_mm > 0) {  // 非零值才有效
                        sum += depth_mm;
                        valid_count++;
                    }
                }
            }
        }
        
        // 至少一半像素有效才认为数据可靠
        int total_pixels = (2 * region_size + 1) * (2 * region_size + 1);
        if (valid_count > total_pixels / 2) {
            float avg_depth_mm = sum / valid_count;
            float distance_m = avg_depth_mm / 1000.0;  // 转换为米
            
            // 数据有效性检查
            bool valid = true;
            
            // 检查1：范围检查
            if (distance_m < depth_min_range_ || distance_m > depth_max_range_) {
                valid = false;
                if (debug_output_) {
                    ROS_WARN_THROTTLE(1, "Depth out of range: %.2f m", distance_m);
                }
            }
            
            // 检查2：突变检查
            if (depth_state_.valid) {
                float delta = std::abs(distance_m - depth_state_.last_distance);
                if (delta > depth_jump_threshold_) {
                    valid = false;
                    if (debug_output_) {
                        ROS_WARN_THROTTLE(1, "Depth jump detected: %.2f -> %.2f m", 
                                         depth_state_.last_distance, distance_m);
                    }
                }
            }
            
            // 更新状态
            depth_state_.distance = distance_m;
            depth_state_.valid = valid;
            depth_state_.last_update = ros::Time::now();
            depth_state_.last_distance = distance_m;
            
            if (debug_output_ && valid) {
                ROS_INFO_THROTTLE(1, "Depth: %.3f m (valid pixels: %d/%d)", 
                                 distance_m, valid_count, total_pixels);
            }
        } else {
            depth_state_.valid = false;
            if (debug_output_) {
                ROS_WARN_THROTTLE(1, "Insufficient valid pixels: %d/%d", 
                                 valid_count, total_pixels);
            }
        }
        
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        depth_state_.valid = false;
    }
}

// 姿态回调（获取yaw角）
void TrajectoryServer::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
    pose_received_ = true;
}

// 检查深度数据有效性
bool TrajectoryServer::checkDepthValid() {
    if (!depth_state_.valid) {
        return false;
    }
    
    // 检查数据超时
    ros::Duration age = ros::Time::now() - depth_state_.last_update;
    if (age.toSec() > depth_timeout_) {
        if (debug_output_) {
            ROS_WARN_THROTTLE(1, "Depth data timeout: %.2f s", age.toSec());
        }
        return false;
    }
    
    return true;
}

// 获取当前yaw角（弧度）
double TrajectoryServer::getCurrentYaw() {
    if (!pose_received_) {
        ROS_WARN_THROTTLE(5, "Pose not received yet, using yaw=0");
        return 0.0;
    }
    
    tf2::Quaternion q(
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z,
        current_pose_.pose.orientation.w
    );
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    return yaw;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_server");
    TrajectoryServer server;
    ros::spin();
    return 0;
}