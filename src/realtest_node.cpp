#include <ros/ros.h>

#include <cmath>
#include <algorithm>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GPSRAW.h>

#include <sensor_msgs/NavSatFix.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// ============================================================================
// 真机空飞稳定性测试节点
//
// 目标：在真实喷涂测试前，验证 OFFBOARD + local position + yaw reference 的稳定性。
// 流程：
//   1. 等待 MAVROS 连接
//   2. 等待 local_position/pose 可用，且位姿数值有效
//   3. 可选等待 RTK Fixed
//   4. 冻结 mission0：EKF 稳定后的当前位置 + yaw
//   5. 发送初始 setpoint
//   6. 切 OFFBOARD，解锁
//   7. 以 0.1 m/s 起飞到 mission0.z + 2m
//   8. 悬停稳定
//   9. 沿 mission0 yaw 的机头方向，以 0.1 m/s 前飞 3m
//  10. 到达后持续悬停
//
// 注意：
//   - 所有目标位置均基于 ROS/MAVROS 的 local ENU/map 语义。
//   - mission0 是本节点冻结的任务参考，不等于 PX4 EKF local origin。
//   - 实机中 /mavros/local_position/pose 的 z=0 不一定等于当前地面高度，
//     所以不能用 abs(z)<某阈值 判断 EKF/local pose 是否可用。
// ============================================================================

mavros_msgs::State g_state;
geometry_msgs::PoseStamped g_pose;
bool g_pose_received = false;

sensor_msgs::NavSatFix g_navsat_fix;
bool g_navsat_received = false;

mavros_msgs::GPSRAW g_gps_raw;
bool g_gps_raw_received = false;

void stateCb(const mavros_msgs::State::ConstPtr& msg) {
    g_state = *msg;
}

void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    g_pose = *msg;
    g_pose_received = true;
}

void navsatCb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    g_navsat_fix = *msg;
    g_navsat_received = true;
}

void gpsRawCb(const mavros_msgs::GPSRAW::ConstPtr& msg) {
    g_gps_raw = *msg;
    g_gps_raw_received = true;
}

double getYawFromQuat(const geometry_msgs::Quaternion& q_msg) {
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

double pointDistance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double angleDiff(double a, double b) {
    double d = std::fabs(a - b);
    while (d > M_PI) {
        d = std::fabs(d - 2.0 * M_PI);
    }
    return d;
}

struct MissionFrame {
    geometry_msgs::Point origin_map;
    double yaw0 = 0.0;
    bool valid = false;
};

void fillYawOrientation(geometry_msgs::PoseStamped& pose, double yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
}

void publishPositionSetpoint(
    ros::Publisher& pub,
    double x,
    double y,
    double z,
    double yaw)
{
    mavros_msgs::PositionTarget target;
    target.header.stamp = ros::Time::now();
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // 只使用 position + yaw
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

    pub.publish(target);
}

void publishPosVelSetpoint(
    ros::Publisher& pub,
    double px,
    double py,
    double pz,
    double vx,
    double vy,
    double vz,
    double yaw)
{
    mavros_msgs::PositionTarget target;
    target.header.stamp = ros::Time::now();
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // 使用 position + velocity + yaw；忽略加速度和 yaw_rate
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

    pub.publish(target);
}

bool waitForConnection(ros::Rate& rate, double timeout_s) {
    ROS_INFO("Waiting for MAVROS/FCU connection...");
    const ros::Time start = ros::Time::now();

    while (ros::ok() && !g_state.connected) {
        ros::spinOnce();
        rate.sleep();

        if ((ros::Time::now() - start).toSec() > timeout_s) {
            ROS_ERROR("Timeout waiting for MAVROS connection");
            return false;
        }
    }

    ROS_INFO("MAVROS connected.");
    return true;
}

// 这里是本次核心修改：
// 不再要求 local z 接近 0。
// 只要求收到 local pose，且位置/姿态数值有效。
// 因为实机 PX4 EKF local origin 不一定等于当前起飞地面。
bool waitForPoseAndEKFReady(ros::Rate& rate, double timeout_s, double z_abs_limit) {
    (void)z_abs_limit;

    ROS_INFO("Waiting for local position data...");
    const ros::Time start = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();

        if (g_pose_received) {
            const auto& p = g_pose.pose.position;
            const auto& q = g_pose.pose.orientation;

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
                const double yaw = getYawFromQuat(g_pose.pose.orientation);

                ROS_INFO("Local pose received: pos=(%.2f, %.2f, %.2f), yaw=%.1f deg. "
                         "mission0 stability will be checked next.",
                         p.x,
                         p.y,
                         p.z,
                         yaw * 180.0 / M_PI);

                return true;
            }

            ROS_WARN_THROTTLE(
                5.0,
                "Waiting for valid local pose: pos=(%.2f, %.2f, %.2f), q_norm=%.3f",
                p.x,
                p.y,
                p.z,
                q_norm
            );
        } else {
            ROS_WARN_THROTTLE(5.0, "Waiting for /mavros/local_position/pose...");
        }

        if ((ros::Time::now() - start).toSec() > timeout_s) {
            if (g_pose_received) {
                ROS_ERROR("Local pose timeout: current pos=(%.2f, %.2f, %.2f)",
                          g_pose.pose.position.x,
                          g_pose.pose.position.y,
                          g_pose.pose.position.z);
            } else {
                ROS_ERROR("Local pose timeout: no pose received");
            }
            return false;
        }

        rate.sleep();
    }

    return false;
}

bool waitForRTKIfRequired(ros::Rate& rate, bool require_rtk, double timeout_s) {
    if (!require_rtk) {
        ROS_INFO("RTK check disabled: require_rtk=false. You must confirm RTK manually if needed.");
        return true;
    }

    ROS_INFO("RTK check enabled. Waiting for RTK Fixed...");
    const ros::Time start = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();

        bool rtk_fixed = false;

        // Preferred: MAVROS GPSRAW, fix_type 6 = RTK Fixed, 5 = RTK Float
        if (g_gps_raw_received) {
            if (g_gps_raw.fix_type == 6) {
                rtk_fixed = true;
                ROS_INFO("RTK Fixed by GPSRAW: fix_type=%u, eph=%u, epv=%u, sats=%u",
                         g_gps_raw.fix_type,
                         g_gps_raw.eph,
                         g_gps_raw.epv,
                         g_gps_raw.satellites_visible);
            }
        }

        // Fallback: NavSatFix GBAS status. This may be less precise than GPSRAW fix_type.
        if (!rtk_fixed && g_navsat_received) {
            if (g_navsat_fix.status.status >= sensor_msgs::NavSatStatus::STATUS_GBAS_FIX) {
                rtk_fixed = true;
                ROS_INFO("RTK/GBAS by NavSatFix: status=%d", g_navsat_fix.status.status);
            }
        }

        if (rtk_fixed) {
            return true;
        }

        const double elapsed = (ros::Time::now() - start).toSec();
        if (elapsed > timeout_s) {
            ROS_ERROR("RTK wait timeout after %.1f s", timeout_s);
            if (g_gps_raw_received) {
                ROS_ERROR("Last GPSRAW: fix_type=%u, eph=%u, epv=%u, sats=%u",
                          g_gps_raw.fix_type,
                          g_gps_raw.eph,
                          g_gps_raw.epv,
                          g_gps_raw.satellites_visible);
            }
            if (g_navsat_received) {
                ROS_ERROR("Last NavSatFix status=%d", g_navsat_fix.status.status);
            }
            return false;
        }

        ROS_INFO_THROTTLE(
            5.0,
            "Waiting RTK... GPSRAW received=%d fix_type=%d, NavSat received=%d status=%d, elapsed=%.0f/%.0f s",
            static_cast<int>(g_gps_raw_received),
            g_gps_raw_received ? static_cast<int>(g_gps_raw.fix_type) : -1,
            static_cast<int>(g_navsat_received),
            g_navsat_received ? g_navsat_fix.status.status : -99,
            elapsed,
            timeout_s
        );

        rate.sleep();
    }

    return false;
}

bool freezeMission0(ros::Rate& rate,
                    int stable_frames_required,
                    double pos_jitter_threshold,
                    double yaw_jitter_threshold,
                    double timeout_s,
                    MissionFrame& mission0)
{
    if (!g_pose_received) {
        ROS_ERROR("Cannot freeze mission0: no pose received");
        return false;
    }

    ROS_INFO("Freezing mission0 reference frame...");

    geometry_msgs::Point prev_pos = g_pose.pose.position;
    double prev_yaw = getYawFromQuat(g_pose.pose.orientation);
    int stable_count = 0;

    const ros::Time start = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();

        const geometry_msgs::Point cur_pos = g_pose.pose.position;
        const double cur_yaw = getYawFromQuat(g_pose.pose.orientation);

        const double pos_diff = pointDistance(prev_pos, cur_pos);
        const double yaw_diff = angleDiff(prev_yaw, cur_yaw);

        if (pos_diff < pos_jitter_threshold && yaw_diff < yaw_jitter_threshold) {
            stable_count++;
        } else {
            stable_count = 0;
        }

        prev_pos = cur_pos;
        prev_yaw = cur_yaw;

        if (stable_count >= stable_frames_required) {
            mission0.origin_map = cur_pos;
            mission0.yaw0 = cur_yaw;
            mission0.valid = true;

            ROS_INFO("mission0 frozen: origin=(%.2f, %.2f, %.2f), yaw=%.1f deg",
                     mission0.origin_map.x,
                     mission0.origin_map.y,
                     mission0.origin_map.z,
                     mission0.yaw0 * 180.0 / M_PI);
            return true;
        }

        const double elapsed = (ros::Time::now() - start).toSec();
        if (elapsed > timeout_s) {
            ROS_WARN("mission0 stability timeout. Freezing current pose.");
            mission0.origin_map = cur_pos;
            mission0.yaw0 = cur_yaw;
            mission0.valid = true;

            ROS_WARN("mission0 frozen by timeout: origin=(%.2f, %.2f, %.2f), yaw=%.1f deg",
                     mission0.origin_map.x,
                     mission0.origin_map.y,
                     mission0.origin_map.z,
                     mission0.yaw0 * 180.0 / M_PI);
            return true;
        }

        ROS_INFO_THROTTLE(
            2.0,
            "mission0 waiting stable: stable_count=%d/%d, pos_diff=%.3f m, yaw_diff=%.2f deg",
            stable_count,
            stable_frames_required,
            pos_diff,
            yaw_diff * 180.0 / M_PI
        );

        rate.sleep();
    }

    return false;
}

bool setMode(ros::ServiceClient& set_mode_client, const std::string& mode) {
    mavros_msgs::SetMode srv;
    srv.request.custom_mode = mode;

    if (set_mode_client.call(srv) && srv.response.mode_sent) {
        ROS_INFO("Mode set to %s", mode.c_str());
        return true;
    }

    ROS_ERROR("Failed to set mode to %s", mode.c_str());
    return false;
}

bool armVehicle(ros::ServiceClient& arming_client) {
    mavros_msgs::CommandBool srv;
    srv.request.value = true;

    if (arming_client.call(srv) && srv.response.success) {
        ROS_INFO("Vehicle armed.");
        return true;
    }

    ROS_ERROR("Failed to arm vehicle.");
    return false;
}

bool flyLinearSegment(
    ros::Publisher& setpoint_pub,
    ros::Rate& rate,
    const geometry_msgs::Point& start,
    const geometry_msgs::Point& goal,
    double yaw,
    double speed,
    double pos_tolerance,
    double extra_wait_s)
{
    const double dx = goal.x - start.x;
    const double dy = goal.y - start.y;
    const double dz = goal.z - start.z;

    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (length < 1e-6) {
        ROS_WARN("flyLinearSegment length near zero. Holding goal.");
        publishPositionSetpoint(setpoint_pub, goal.x, goal.y, goal.z, yaw);
        return true;
    }

    if (speed <= 0.0) {
        ROS_ERROR("Invalid speed: %.3f", speed);
        return false;
    }

    const double duration = length / speed;

    const double ux = dx / length;
    const double uy = dy / length;
    const double uz = dz / length;

    ROS_INFO("Linear segment: length=%.2f m, speed=%.2f m/s, duration=%.1f s",
             length, speed, duration);

    const ros::Time start_time = ros::Time::now();

    while (ros::ok()) {
        const double elapsed = (ros::Time::now() - start_time).toSec();

        double t = elapsed;
        if (t > duration) {
            t = duration;
        }

        const double ratio = t / duration;

        const double cmd_x = start.x + ratio * dx;
        const double cmd_y = start.y + ratio * dy;
        const double cmd_z = start.z + ratio * dz;

        const bool still_moving = elapsed < duration;
        const double vx = still_moving ? speed * ux : 0.0;
        const double vy = still_moving ? speed * uy : 0.0;
        const double vz = still_moving ? speed * uz : 0.0;

        publishPosVelSetpoint(
            setpoint_pub,
            cmd_x,
            cmd_y,
            cmd_z,
            vx,
            vy,
            vz,
            yaw
        );

        ros::spinOnce();

        geometry_msgs::Point current = g_pose.pose.position;
        const double err = pointDistance(current, goal);

        ROS_INFO_THROTTLE(
            2.0,
            "Segment progress: %.1f/%.1f s, cmd=(%.2f, %.2f, %.2f), current=(%.2f, %.2f, %.2f), err=%.2f",
            elapsed,
            duration,
            cmd_x,
            cmd_y,
            cmd_z,
            current.x,
            current.y,
            current.z,
            err
        );

        if (elapsed >= duration && err < pos_tolerance) {
            ROS_INFO("Segment reached: current=(%.2f, %.2f, %.2f), target=(%.2f, %.2f, %.2f), err=%.2f",
                     current.x, current.y, current.z,
                     goal.x, goal.y, goal.z,
                     err);
            return true;
        }

        if (elapsed > duration + extra_wait_s) {
            ROS_ERROR("Segment timeout: target not reached. err=%.2f", err);
            return false;
        }

        rate.sleep();
    }

    return false;
}

void holdPosition(
    ros::Publisher& setpoint_pub,
    ros::Rate& rate,
    const geometry_msgs::Point& hold_point,
    double yaw,
    double duration_s)
{
    const ros::Time start = ros::Time::now();

    while (ros::ok() && (ros::Time::now() - start).toSec() < duration_s) {
        publishPositionSetpoint(
            setpoint_pub,
            hold_point.x,
            hold_point.y,
            hold_point.z,
            yaw
        );

        ros::spinOnce();

        ROS_INFO_THROTTLE(
            2.0,
            "Holding: %.1f/%.1f s, current=(%.2f, %.2f, %.2f), target=(%.2f, %.2f, %.2f)",
            (ros::Time::now() - start).toSec(),
            duration_s,
            g_pose.pose.position.x,
            g_pose.pose.position.y,
            g_pose.pose.position.z,
            hold_point.x,
            hold_point.y,
            hold_point.z
        );

        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "realtest_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string mavros_ns;
    bool require_rtk;
    double rtk_wait_timeout;

    double takeoff_height;
    double forward_distance;
    double command_speed;

    double hover_after_takeoff_s;
    double pos_tolerance;

    double connection_timeout_s;
    double ekf_timeout_s;
    double ekf_z_abs_limit;

    int stable_frames_required;
    double pos_jitter_threshold;
    double yaw_jitter_threshold;
    double mission0_freeze_timeout_s;

    nh_private.param<std::string>("mavros_ns", mavros_ns, "/mavros");

    nh_private.param<bool>("require_rtk", require_rtk, false);
    nh_private.param<double>("rtk_wait_timeout", rtk_wait_timeout, 300.0);

    nh_private.param<double>("takeoff_height", takeoff_height, 2.0);
    nh_private.param<double>("forward_distance", forward_distance, 3.0);
    nh_private.param<double>("command_speed", command_speed, 0.1);

    nh_private.param<double>("hover_after_takeoff_s", hover_after_takeoff_s, 2.0);
    nh_private.param<double>("pos_tolerance", pos_tolerance, 0.20);

    nh_private.param<double>("connection_timeout_s", connection_timeout_s, 30.0);
    nh_private.param<double>("ekf_timeout_s", ekf_timeout_s, 30.0);
    nh_private.param<double>("ekf_z_abs_limit", ekf_z_abs_limit, 2.0);

    nh_private.param<int>("stable_frames_required", stable_frames_required, 20);
    nh_private.param<double>("pos_jitter_threshold", pos_jitter_threshold, 0.05);
    nh_private.param<double>("yaw_jitter_threshold", yaw_jitter_threshold, 0.05);
    nh_private.param<double>("mission0_freeze_timeout_s", mission0_freeze_timeout_s, 15.0);

    ROS_INFO("=== Real Flight Empty Test Configuration ===");
    ROS_INFO("mavros_ns: %s", mavros_ns.c_str());
    ROS_INFO("takeoff_height: %.2f m relative to mission0", takeoff_height);
    ROS_INFO("forward_distance: %.2f m", forward_distance);
    ROS_INFO("command_speed: %.2f m/s", command_speed);
    ROS_INFO("require_rtk: %s", require_rtk ? "true" : "false");

    ros::Subscriber state_sub =
        nh.subscribe(mavros_ns + "/state", 10, stateCb);

    ros::Subscriber pose_sub =
        nh.subscribe(mavros_ns + "/local_position/pose", 10, poseCb);

    ros::Subscriber navsat_sub =
        nh.subscribe(mavros_ns + "/global_position/raw/fix", 10, navsatCb);

    ros::Subscriber gpsraw_sub =
        nh.subscribe(mavros_ns + "/gpsstatus/gps1/raw", 10, gpsRawCb);

    ros::Publisher setpoint_pub =
        nh.advertise<mavros_msgs::PositionTarget>(mavros_ns + "/setpoint_raw/local", 10);

    ros::ServiceClient arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>(mavros_ns + "/cmd/arming");

    ros::ServiceClient set_mode_client =
        nh.serviceClient<mavros_msgs::SetMode>(mavros_ns + "/set_mode");

    ros::Rate rate(20.0);

    // 1. 等待 MAVROS 连接
    if (!waitForConnection(rate, connection_timeout_s)) {
        return 1;
    }

    // 2. 等待 local pose / EKF 基本可用
    // 注意：不再要求 local z 接近 0。
    if (!waitForPoseAndEKFReady(rate, ekf_timeout_s, ekf_z_abs_limit)) {
        return 1;
    }

    // 3. 可选 RTK 检查
    // 默认 require_rtk=false。若你手动确认 RTK fixed 后再启动本节点，可以继续保持 false。
    if (!waitForRTKIfRequired(rate, require_rtk, rtk_wait_timeout)) {
        return 1;
    }

    // 4. 冻结 mission0
    MissionFrame mission0;
    if (!freezeMission0(
            rate,
            stable_frames_required,
            pos_jitter_threshold,
            yaw_jitter_threshold,
            mission0_freeze_timeout_s,
            mission0))
    {
        return 1;
    }

    if (!mission0.valid) {
        ROS_ERROR("mission0 invalid after freeze");
        return 1;
    }

    // 5. 计算起飞目标：相对 mission0 上升 2m
    geometry_msgs::Point takeoff_start = mission0.origin_map;
    geometry_msgs::Point takeoff_goal = mission0.origin_map;
    takeoff_goal.z = mission0.origin_map.z + takeoff_height;

    ROS_INFO("Takeoff target: mission0.z %.2f + %.2f = %.2f",
             mission0.origin_map.z,
             takeoff_height,
             takeoff_goal.z);

    // 6. OFFBOARD 前先持续发送初始 setpoint
    ROS_INFO("Sending initial setpoints before OFFBOARD...");
    for (int i = 0; ros::ok() && i < 100; ++i) {
        publishPositionSetpoint(
            setpoint_pub,
            takeoff_start.x,
            takeoff_start.y,
            takeoff_goal.z,
            mission0.yaw0
        );
        ros::spinOnce();
        rate.sleep();
    }

    // 7. 切 OFFBOARD
    if (!setMode(set_mode_client, "OFFBOARD")) {
        return 1;
    }

    ros::Duration(0.5).sleep();

    // 8. 解锁
    if (!armVehicle(arming_client)) {
        return 1;
    }

    // 9. 起飞段：以 0.1 m/s 从 mission0.z 到 mission0.z + 2m
    ROS_INFO("Taking off %.2f m at %.2f m/s...", takeoff_height, command_speed);
    if (!flyLinearSegment(
            setpoint_pub,
            rate,
            takeoff_start,
            takeoff_goal,
            mission0.yaw0,
            command_speed,
            pos_tolerance,
            20.0))
    {
        ROS_ERROR("Takeoff segment failed.");
        return 1;
    }

    // 10. 起飞后短暂悬停
    ROS_INFO("Hovering after takeoff for %.1f s...", hover_after_takeoff_s);
    holdPosition(
        setpoint_pub,
        rate,
        takeoff_goal,
        mission0.yaw0,
        hover_after_takeoff_s
    );

    // 11. 沿 mission0 yaw 的机头方向前飞 3m，速度 0.1 m/s
    // 起点取当前实际位置，避免起飞阶段小误差造成指令突变。
    geometry_msgs::Point forward_start = g_pose.pose.position;
    forward_start.z = takeoff_goal.z;

    const double dir_x = std::cos(mission0.yaw0);
    const double dir_y = std::sin(mission0.yaw0);

    geometry_msgs::Point forward_goal = forward_start;
    forward_goal.x += forward_distance * dir_x;
    forward_goal.y += forward_distance * dir_y;
    forward_goal.z = takeoff_goal.z;

    ROS_INFO("Flying forward %.2f m at %.2f m/s along mission0 yaw %.1f deg",
             forward_distance,
             command_speed,
             mission0.yaw0 * 180.0 / M_PI);
    ROS_INFO("Forward start=(%.2f, %.2f, %.2f), goal=(%.2f, %.2f, %.2f)",
             forward_start.x, forward_start.y, forward_start.z,
             forward_goal.x, forward_goal.y, forward_goal.z);

    if (!flyLinearSegment(
            setpoint_pub,
            rate,
            forward_start,
            forward_goal,
            mission0.yaw0,
            command_speed,
            pos_tolerance,
            20.0))
    {
        ROS_ERROR("Forward segment failed.");
        return 1;
    }

    // 12. 到达后持续悬停
    ROS_INFO("=== Empty flight test complete. Holding final position. Ctrl+C or switch mode to exit. ===");

    while (ros::ok()) {
        publishPositionSetpoint(
            setpoint_pub,
            forward_goal.x,
            forward_goal.y,
            forward_goal.z,
            mission0.yaw0
        );

        ros::spinOnce();

        ROS_INFO_THROTTLE(
            5.0,
            "Holding final: current=(%.2f, %.2f, %.2f), target=(%.2f, %.2f, %.2f)",
            g_pose.pose.position.x,
            g_pose.pose.position.y,
            g_pose.pose.position.z,
            forward_goal.x,
            forward_goal.y,
            forward_goal.z
        );

        rate.sleep();
    }

    return 0;
}