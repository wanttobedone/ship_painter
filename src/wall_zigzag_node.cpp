#include <ros/ros.h>

#include <cmath>
#include <algorithm>
#include <string>
#include <vector>

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
// 外墙 Zigzag 展示飞行节点
//
// 任务目的：
//   用于真实外墙前的展示视频录制，不执行喷涂路径规划。
//   只验证并展示：mission0 冻结、相对 mission0 起飞、面向墙面保持 yaw、
//   在墙面前执行上下 zigzag / 割草机路径。
//
// 默认轨迹：
//   1. 冻结 mission0：当前位置 + 当前 yaw
//   2. 起飞到 mission0.z + 1m
//   3. 沿机头方向前飞 1m
//   4. 上飞 5m
//   5. 向右 2m
//   6. 下飞 5m
//   7. 向右 2m
//   8. 上飞 5m
//   9. 向右 2m
//  10. 悬停
//
// 坐标约定：
//   mission0.x：冻结时无人机机头方向
//   mission0.z：世界竖直向上
//   mission0.y：左侧方向
//   因此“无人机/人面向墙时的右边”是 mission0.y 的负方向。
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
    return std::fabs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

struct MissionFrame {
    geometry_msgs::Point origin_map;
    double yaw0 = 0.0;
    bool valid = false;
};

geometry_msgs::Point missionLocalToMap(
    const MissionFrame& mission0,
    double x_local,
    double y_local,
    double z_local)
{
    geometry_msgs::Point p;

    const double c = std::cos(mission0.yaw0);
    const double s = std::sin(mission0.yaw0);

    // Rz(yaw0) * [x_local, y_local, z_local]^T
    p.x = mission0.origin_map.x + c * x_local - s * y_local;
    p.y = mission0.origin_map.y + s * x_local + c * y_local;
    p.z = mission0.origin_map.z + z_local;

    return p;
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

    // 使用 position + velocity + yaw；忽略 acceleration 和 yaw_rate
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

bool waitForPoseAndEKFReady(ros::Rate& rate, double timeout_s, double z_abs_limit) {
    ROS_INFO("Waiting for local position data and EKF readiness...");
    const ros::Time start = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();

        if (g_pose_received) {
            const double z = g_pose.pose.position.z;

            if (std::fabs(z) < z_abs_limit) {
                ROS_INFO("EKF/local pose ready: pos=(%.2f, %.2f, %.2f), yaw=%.1f deg",
                         g_pose.pose.position.x,
                         g_pose.pose.position.y,
                         g_pose.pose.position.z,
                         getYawFromQuat(g_pose.pose.orientation) * 180.0 / M_PI);
                return true;
            }

            ROS_WARN_THROTTLE(
                5.0,
                "Waiting for EKF/local pose: current z=%.2f, expected |z| < %.2f",
                z,
                z_abs_limit
            );
        }

        if ((ros::Time::now() - start).toSec() > timeout_s) {
            if (g_pose_received) {
                ROS_ERROR("EKF/local pose timeout: current pos=(%.2f, %.2f, %.2f)",
                          g_pose.pose.position.x,
                          g_pose.pose.position.y,
                          g_pose.pose.position.z);
            } else {
                ROS_ERROR("EKF/local pose timeout: no pose received");
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

        // Preferred: GPSRAW fix_type 6 = RTK Fixed, 5 = RTK Float
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

        // Fallback: NavSatFix GBAS
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

bool freezeMission0(
    ros::Rate& rate,
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

void holdPosition(
    ros::Publisher& setpoint_pub,
    ros::Rate& rate,
    const geometry_msgs::Point& hold_point,
    double yaw,
    double duration_s)
{
    if (duration_s <= 0.0) {
        return;
    }

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
            "Holding corner: %.1f/%.1f s, current=(%.2f, %.2f, %.2f), target=(%.2f, %.2f, %.2f)",
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
        ROS_WARN("Segment length near zero. Holding goal.");
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

    ROS_INFO("Linear segment: start=(%.2f, %.2f, %.2f), goal=(%.2f, %.2f, %.2f), length=%.2f m, speed=%.2f m/s, duration=%.1f s",
             start.x, start.y, start.z,
             goal.x, goal.y, goal.z,
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

        const geometry_msgs::Point current = g_pose.pose.position;
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

std::vector<geometry_msgs::Point> buildZigzagWaypoints(
    const MissionFrame& mission0,
    double takeoff_height,
    double approach_distance,
    double vertical_span,
    double lateral_step,
    int vertical_passes,
    int lateral_moves,
    int lateral_direction)
{
    std::vector<geometry_msgs::Point> waypoints;

    // 起飞点：mission0 正上方 takeoff_height
    waypoints.push_back(
        missionLocalToMap(mission0, 0.0, 0.0, takeoff_height)
    );

    // 前飞到墙前展示距离
    double x = approach_distance;
    double y = 0.0;
    double z = takeoff_height;

    waypoints.push_back(
        missionLocalToMap(mission0, x, y, z)
    );

    // Zigzag 扫描
    // pass 0: up
    // pass 1: down
    // pass 2: up
    // ...
    for (int pass = 0; pass < vertical_passes; ++pass) {
        const bool go_up = (pass % 2 == 0);

        if (go_up) {
            z = takeoff_height + vertical_span;
        } else {
            z = takeoff_height;
        }

        waypoints.push_back(
            missionLocalToMap(mission0, x, y, z)
        );

        if (pass < lateral_moves) {
            y += static_cast<double>(lateral_direction) * lateral_step;

            waypoints.push_back(
                missionLocalToMap(mission0, x, y, z)
            );
        }
    }

    return waypoints;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_zigzag_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string mavros_ns;
    bool require_rtk;
    double rtk_wait_timeout;

    double takeoff_height;
    double approach_distance;
    double vertical_span;
    double lateral_step;
    int vertical_passes;
    int lateral_moves;
    int lateral_direction;

    double command_speed;
    double corner_hold_s;
    double final_hold_s;
    bool hold_final_forever;

    double pos_tolerance;
    double segment_extra_wait_s;

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

    nh_private.param<double>("takeoff_height", takeoff_height, 1.0);
    nh_private.param<double>("approach_distance", approach_distance, 1.0);
    nh_private.param<double>("vertical_span", vertical_span, 5.0);
    nh_private.param<double>("lateral_step", lateral_step, 2.0);
    nh_private.param<int>("vertical_passes", vertical_passes, 3);
    nh_private.param<int>("lateral_moves", lateral_moves, 3);

    // -1 表示无人机/人面向墙时的右边；+1 表示左边
    nh_private.param<int>("lateral_direction", lateral_direction, -1);

    nh_private.param<double>("command_speed", command_speed, 0.2);
    nh_private.param<double>("corner_hold_s", corner_hold_s, 0.5);
    nh_private.param<double>("final_hold_s", final_hold_s, 10.0);
    nh_private.param<bool>("hold_final_forever", hold_final_forever, true);

    nh_private.param<double>("pos_tolerance", pos_tolerance, 0.25);
    nh_private.param<double>("segment_extra_wait_s", segment_extra_wait_s, 20.0);

    nh_private.param<double>("connection_timeout_s", connection_timeout_s, 30.0);
    nh_private.param<double>("ekf_timeout_s", ekf_timeout_s, 30.0);
    nh_private.param<double>("ekf_z_abs_limit", ekf_z_abs_limit, 2.0);

    nh_private.param<int>("stable_frames_required", stable_frames_required, 20);
    nh_private.param<double>("pos_jitter_threshold", pos_jitter_threshold, 0.05);
    nh_private.param<double>("yaw_jitter_threshold", yaw_jitter_threshold, 0.05);
    nh_private.param<double>("mission0_freeze_timeout_s", mission0_freeze_timeout_s, 15.0);

    if (vertical_passes <= 0) {
        ROS_ERROR("vertical_passes must be > 0");
        return 1;
    }

    if (lateral_moves < 0) {
        ROS_ERROR("lateral_moves must be >= 0");
        return 1;
    }

    if (lateral_direction != -1 && lateral_direction != 1) {
        ROS_WARN("lateral_direction should be -1 or 1. Current=%d, forcing to -1", lateral_direction);
        lateral_direction = -1;
    }

    ROS_INFO("=== Wall Zigzag Demo Configuration ===");
    ROS_INFO("mavros_ns: %s", mavros_ns.c_str());
    ROS_INFO("takeoff_height: %.2f m relative to mission0", takeoff_height);
    ROS_INFO("approach_distance: %.2f m forward", approach_distance);
    ROS_INFO("vertical_span: %.2f m", vertical_span);
    ROS_INFO("lateral_step: %.2f m", lateral_step);
    ROS_INFO("vertical_passes: %d", vertical_passes);
    ROS_INFO("lateral_moves: %d", lateral_moves);
    ROS_INFO("lateral_direction: %d (-1=right, +1=left)", lateral_direction);
    ROS_INFO("command_speed: %.2f m/s", command_speed);
    ROS_INFO("corner_hold_s: %.2f s", corner_hold_s);
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

    // 1. MAVROS connection
    if (!waitForConnection(rate, connection_timeout_s)) {
        return 1;
    }

    // 2. local pose / EKF basic readiness
    if (!waitForPoseAndEKFReady(rate, ekf_timeout_s, ekf_z_abs_limit)) {
        return 1;
    }

    // 3. optional RTK check
    if (!waitForRTKIfRequired(rate, require_rtk, rtk_wait_timeout)) {
        return 1;
    }

    // 4. freeze mission0
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

    // 5. build waypoint list in map frame
    std::vector<geometry_msgs::Point> waypoints = buildZigzagWaypoints(
        mission0,
        takeoff_height,
        approach_distance,
        vertical_span,
        lateral_step,
        vertical_passes,
        lateral_moves,
        lateral_direction
    );

    if (waypoints.empty()) {
        ROS_ERROR("No waypoints generated");
        return 1;
    }

    ROS_INFO("Generated %zu waypoints:", waypoints.size());
    for (size_t i = 0; i < waypoints.size(); ++i) {
        ROS_INFO("  WP%zu: map=(%.2f, %.2f, %.2f)",
                 i,
                 waypoints[i].x,
                 waypoints[i].y,
                 waypoints[i].z);
    }

    // 6. pre-stream setpoint before OFFBOARD
    ROS_INFO("Sending initial setpoints before OFFBOARD...");
    const geometry_msgs::Point first_wp = waypoints.front();

    for (int i = 0; ros::ok() && i < 100; ++i) {
        publishPositionSetpoint(
            setpoint_pub,
            first_wp.x,
            first_wp.y,
            first_wp.z,
            mission0.yaw0
        );
        ros::spinOnce();
        rate.sleep();
    }

    // 7. switch OFFBOARD
    if (!setMode(set_mode_client, "OFFBOARD")) {
        return 1;
    }

    ros::Duration(0.5).sleep();

    // 8. arm
    if (!armVehicle(arming_client)) {
        return 1;
    }

    // 9. execute waypoint segments
    geometry_msgs::Point current_start = mission0.origin_map;

    for (size_t i = 0; i < waypoints.size(); ++i) {
        const geometry_msgs::Point goal = waypoints[i];

        ROS_INFO("Executing segment %zu/%zu", i + 1, waypoints.size());

        if (!flyLinearSegment(
                setpoint_pub,
                rate,
                current_start,
                goal,
                mission0.yaw0,
                command_speed,
                pos_tolerance,
                segment_extra_wait_s))
        {
            ROS_ERROR("Segment %zu failed. Holding current position.", i + 1);
            return 1;
        }

        holdPosition(
            setpoint_pub,
            rate,
            goal,
            mission0.yaw0,
            corner_hold_s
        );

        // 用目标点作为下一段起点，使轨迹几何严格连续。
        // 实际位置误差由 flyLinearSegment 的 pos_tolerance 控制。
        current_start = goal;
    }

    const geometry_msgs::Point final_wp = waypoints.back();

    ROS_INFO("=== Wall zigzag demo complete. Final hold. ===");

    if (hold_final_forever) {
        while (ros::ok()) {
            publishPositionSetpoint(
                setpoint_pub,
                final_wp.x,
                final_wp.y,
                final_wp.z,
                mission0.yaw0
            );

            ros::spinOnce();

            ROS_INFO_THROTTLE(
                5.0,
                "Holding final forever: current=(%.2f, %.2f, %.2f), target=(%.2f, %.2f, %.2f), yaw=%.1f deg",
                g_pose.pose.position.x,
                g_pose.pose.position.y,
                g_pose.pose.position.z,
                final_wp.x,
                final_wp.y,
                final_wp.z,
                mission0.yaw0 * 180.0 / M_PI
            );

            rate.sleep();
        }
    } else {
        holdPosition(
            setpoint_pub,
            rate,
            final_wp,
            mission0.yaw0,
            final_hold_s
        );
    }

    return 0;
}