#include <ros/ros.h>
#include "ship_painter/Bspline.h"
#include "ship_painter/BsplineLayer.h"
#include "ship_painter/bspline.h"
#include <mavros_msgs/PositionTarget.h>
#include <visualization_msgs/Marker.h>

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
        
        // 计算yaw（保持机头朝向表面）
        Eigen::Vector3d spray_dir = -n;
        double yaw = std::atan2(spray_dir.y(), spray_dir.x());
        
        // 发布指令
        mavros_msgs::PositionTarget target;
        target.header.stamp = ros::Time::now();
        target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        
        target.position.x = p.x();
        target.position.y = p.y();
        target.position.z = p.z();
        
        target.velocity.x = v.x();
        target.velocity.y = v.y();
        target.velocity.z = v.z();
        
        target.yaw = yaw;
        target.yaw_rate = 0.0;
        
        target.type_mask = 
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ;

            //打印当前状态
        ROS_INFO_THROTTLE(1.0, "Layer %zu/%zu, t=%.2f/%.2f, pos=[%.2f, %.2f, %.2f]",
                        current_layer_idx_ + 1, layers_.size(),
                        t, current_traj.getTotalTime(),
                        p.x(), p.y(), p.z());

        // ========== 布可视化 ==========
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_server");
    TrajectoryServer server;
    ros::spin();
    return 0;
}