/*
 * VINS to MAVROS Bridge Node
 *
 * Purpose: Bridge VINS-Fusion odometry output to MAVROS vision_pose input
 *          to enable PX4 EKF2 vision-aided navigation
 *
 * Data Flow:
 *   Subscribe: /vins_estimator/odometry (nav_msgs::Odometry)
 *   Publish:   /mavros/vision_pose/pose (geometry_msgs::PoseStamped)
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

class VinsToMavrosBridge {
public:
    VinsToMavrosBridge() : nh_("~") {
        // 从参数服务器读取配置
        nh_.param<std::string>("vins_odom_topic", vins_odom_topic_, "/vins_estimator/odometry");
        nh_.param<std::string>("mavros_vision_topic", mavros_vision_topic_, "/mavros/vision_pose/pose");
        nh_.param<std::string>("output_frame_id", output_frame_id_, "map");

        // 订阅 VINS 里程计
        vins_sub_ = nh_.subscribe(vins_odom_topic_, 10,
                                  &VinsToMavrosBridge::vinsOdomCallback, this);

        // 发布 MAVROS 视觉位姿
        mavros_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(mavros_vision_topic_, 10);

        ROS_INFO("[VINS-MAVROS Bridge] Initialized");
        ROS_INFO("  Subscribing: %s", vins_odom_topic_.c_str());
        ROS_INFO("  Publishing:  %s", mavros_vision_topic_.c_str());
        ROS_INFO("  Frame ID:    %s", output_frame_id_.c_str());
    }

private:
    void vinsOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 创建 PoseStamped 消息
        geometry_msgs::PoseStamped vision_pose;

        // 复制时间戳和坐标系
        vision_pose.header.stamp = msg->header.stamp;
        vision_pose.header.frame_id = output_frame_id_;

        // 复制位置和姿态（从 odometry 的 pose.pose）
        vision_pose.pose = msg->pose.pose;

        // 发布到 MAVROS
        mavros_pub_.publish(vision_pose);

        // 定期打印调试信息（每1秒）
        static ros::Time last_print = ros::Time::now();
        if ((ros::Time::now() - last_print).toSec() > 1.0) {
            ROS_INFO_THROTTLE(5.0, "[Bridge] VINS -> MAVROS | Pos: [%.2f, %.2f, %.2f]",
                             vision_pose.pose.position.x,
                             vision_pose.pose.position.y,
                             vision_pose.pose.position.z);
            last_print = ros::Time::now();
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber vins_sub_;
    ros::Publisher mavros_pub_;

    std::string vins_odom_topic_;
    std::string mavros_vision_topic_;
    std::string output_frame_id_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vins_to_mavros_bridge");

    VinsToMavrosBridge bridge;

    ROS_INFO("[VINS-MAVROS Bridge] Spinning...");
    ros::spin();

    return 0;
}
