#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

// 真机飞行测试节点
// 流程：(等RTK) → 起飞2m → 悬停15s → 向机头前方以0.2m/s飞3m → 悬停保持

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
bool pose_received = false;
int gps_fix_type = -1;
bool gps_received = false;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
    pose_received = true;
}

void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    gps_fix_type = msg->status.status;
    gps_received = true;
}

double getYaw(const geometry_msgs::Quaternion& q) {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    return yaw;
}

double dist(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "realtest_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // 参数
    bool require_rtk;
    double rtk_wait_timeout;
    nh_private.param("require_rtk", require_rtk, false);
    nh_private.param("rtk_wait_timeout", rtk_wait_timeout, 300.0);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(
        "/mavros/global_position/raw/fix", 10, gps_cb);
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
        "/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        "/mavros/set_mode");

    ros::Rate rate(20.0);
    double dt = 1.0 / 20.0;

    // 辅助：发布位置setpoint（悬停用）
    auto publishPosition = [&](double x, double y, double z, double yaw) {
        mavros_msgs::PositionTarget target;
        target.header.stamp = ros::Time::now();
        target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target.type_mask =
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;
        target.yaw = yaw;
        setpoint_pub.publish(target);
    };

    // 辅助：发布位置+速度setpoint（匀速移动用）
    auto publishPosVel = [&](double px, double py, double pz,
                             double vx, double vy, double vz, double yaw) {
        mavros_msgs::PositionTarget target;
        target.header.stamp = ros::Time::now();
        target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target.type_mask =
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        target.position.x = px;
        target.position.y = py;
        target.position.z = pz;
        target.velocity.x = vx;
        target.velocity.y = vy;
        target.velocity.z = vz;
        target.yaw = yaw;
        setpoint_pub.publish(target);
    };

    // 1. 等待MAVROS连接
    ROS_INFO("Waiting for FCU connection...");
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected.");

    // 2. 等待位置数据
    ROS_INFO("Waiting for pose data...");
    while (ros::ok() && !pose_received) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Pose received: (%.2f, %.2f, %.2f)",
             current_pose.pose.position.x,
             current_pose.pose.position.y,
             current_pose.pose.position.z);

    // 3. RTK等待
    if (require_rtk) {
        ROS_INFO("=== RTK mode enabled, waiting for RTK Fix... ===");
        ros::Time rtk_start = ros::Time::now();

        while (ros::ok()) {
            ros::spinOnce();

            if (gps_received && gps_fix_type >= sensor_msgs::NavSatStatus::STATUS_GBAS_FIX) {
                ROS_INFO("RTK Fixed! GPS status=%d", gps_fix_type);
                break;
            }

            double elapsed = (ros::Time::now() - rtk_start).toSec();
            if (elapsed > rtk_wait_timeout) {
                ROS_ERROR("RTK timeout (%.0fs)! Aborting.", rtk_wait_timeout);
                return 1;
            }

            const char* status_str = "NO_DATA";
            if (gps_received) {
                switch (gps_fix_type) {
                    case -1: status_str = "NO_FIX"; break;
                    case 0:  status_str = "GPS_FIX"; break;
                    case 1:  status_str = "SBAS/RTK_FLOAT"; break;
                    default: status_str = "UNKNOWN"; break;
                }
            }
            ROS_INFO_THROTTLE(5, "Waiting RTK... status=%s, elapsed=%.0f/%.0fs",
                              status_str, elapsed, rtk_wait_timeout);
            rate.sleep();
        }
    } else {
        ROS_INFO("RTK not required (require_rtk=false).");
    }

    // 记录起飞点（RTK收敛后的精确位置）
    ros::spinOnce();
    geometry_msgs::PoseStamped home_pose = current_pose;
    double home_yaw = getYaw(current_pose.pose.orientation);
    double takeoff_z = home_pose.pose.position.z + 2.0;
    ROS_INFO("Home (post-RTK): (%.2f, %.2f, %.2f), yaw=%.1f deg",
             home_pose.pose.position.x,
             home_pose.pose.position.y,
             home_pose.pose.position.z,
             home_yaw * 180.0 / M_PI);

    // 4. 发送初始setpoint（OFFBOARD切换前必须先发）
    ROS_INFO("Sending initial setpoints...");
    for (int i = 0; ros::ok() && i < 100; ++i) {
        publishPosition(home_pose.pose.position.x,
                        home_pose.pose.position.y,
                        takeoff_z, home_yaw);
        ros::spinOnce();
        rate.sleep();
    }

    // 5. 切换OFFBOARD + 解锁
    mavros_msgs::SetMode offb_mode;
    offb_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (set_mode_client.call(offb_mode) && offb_mode.response.mode_sent) {
        ROS_INFO("OFFBOARD mode set.");
    } else {
        ROS_ERROR("Failed to set OFFBOARD mode!");
        return 1;
    }

    ros::Duration(0.5).sleep();

    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed.");
    } else {
        ROS_ERROR("Failed to arm!");
        return 1;
    }

    // 6. 起飞到2m，等待到达
    ROS_INFO("Taking off to 2m...");
    double pos_tolerance = 0.15;
    while (ros::ok()) {
        publishPosition(home_pose.pose.position.x,
                        home_pose.pose.position.y,
                        takeoff_z, home_yaw);
        ros::spinOnce();
        rate.sleep();

        geometry_msgs::Point target_pt;
        target_pt.x = home_pose.pose.position.x;
        target_pt.y = home_pose.pose.position.y;
        target_pt.z = takeoff_z;
        if (pose_received && dist(current_pose.pose.position, target_pt) < pos_tolerance) {
            ROS_INFO("Reached takeoff altitude: (%.2f, %.2f, %.2f)",
                     current_pose.pose.position.x,
                     current_pose.pose.position.y,
                     current_pose.pose.position.z);
            break;
        }
    }

    // 7. 悬停15秒
    ROS_INFO("Hovering for 15 seconds...");
    ros::Time hover_start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - hover_start).toSec() < 15.0) {
        publishPosition(home_pose.pose.position.x,
                        home_pose.pose.position.y,
                        takeoff_z, home_yaw);
        ros::spinOnce();
        rate.sleep();

        ROS_INFO_THROTTLE(3, "Hovering... %.0f/15s, pos=(%.2f, %.2f, %.2f)",
                          (ros::Time::now() - hover_start).toSec(),
                          current_pose.pose.position.x,
                          current_pose.pose.position.y,
                          current_pose.pose.position.z);
    }
    ROS_INFO("Hover complete.");

    // 8. 以0.2m/s向机头前方飞3m（逐帧插值位置+速度前馈）
    double fly_speed = 0.2;  // m/s
    double fly_dist = 3.0;   // m
    double fly_duration = fly_dist / fly_speed; // 15秒

    double dir_x = cos(home_yaw);
    double dir_y = sin(home_yaw);

    // 起始点取当前实际位置
    double start_x = current_pose.pose.position.x;
    double start_y = current_pose.pose.position.y;
    double end_x = start_x + fly_dist * dir_x;
    double end_y = start_y + fly_dist * dir_y;

    ROS_INFO("Flying 3m forward at 0.2m/s (%.1fs), target=(%.2f, %.2f, %.2f)",
             fly_duration, end_x, end_y, takeoff_z);

    ros::Time fly_start = ros::Time::now();
    while (ros::ok()) {
        double t = (ros::Time::now() - fly_start).toSec();
        if (t >= fly_duration) t = fly_duration;

        // 线性插值位置
        double frac = t / fly_duration;
        double cmd_x = start_x + frac * fly_dist * dir_x;
        double cmd_y = start_y + frac * fly_dist * dir_y;

        // 速度前馈（到达后速度归零）
        double vx = (t < fly_duration) ? fly_speed * dir_x : 0.0;
        double vy = (t < fly_duration) ? fly_speed * dir_y : 0.0;

        publishPosVel(cmd_x, cmd_y, takeoff_z, vx, vy, 0.0, home_yaw);
        ros::spinOnce();
        rate.sleep();

        ROS_INFO_THROTTLE(2, "Forward: %.1f/%.1fs, pos=(%.2f, %.2f, %.2f), cmd=(%.2f, %.2f)",
                          t, fly_duration,
                          current_pose.pose.position.x,
                          current_pose.pose.position.y,
                          current_pose.pose.position.z,
                          cmd_x, cmd_y);

        // 时间到达且位置接近目标
        if (t >= fly_duration) {
            geometry_msgs::Point end_pt;
            end_pt.x = end_x; end_pt.y = end_y; end_pt.z = takeoff_z;
            if (dist(current_pose.pose.position, end_pt) < pos_tolerance) {
                ROS_INFO("Reached forward position: (%.2f, %.2f, %.2f)",
                         current_pose.pose.position.x,
                         current_pose.pose.position.y,
                         current_pose.pose.position.z);
                break;
            }
        }
    }

    // 9. 悬停保持
    ROS_INFO("=== Holding position. Ctrl+C or switch mode to exit. ===");
    while (ros::ok()) {
        publishPosition(end_x, end_y, takeoff_z, home_yaw);
        ros::spinOnce();
        rate.sleep();

        ROS_INFO_THROTTLE(5, "Holding at (%.2f, %.2f, %.2f)",
                          current_pose.pose.position.x,
                          current_pose.pose.position.y,
                          current_pose.pose.position.z);
    }

    return 0;
}
