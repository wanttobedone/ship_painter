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
// 外墙 Horizontal Zigzag 展示飞行节点
//
// 任务目的：
//   用于真实外墙前的展示视频录制，不执行喷涂路径规划。
//   与 wall_zigzag_node 保持同一套 mission0 工作流：
//     - 等 MAVROS
//     - 等 local pose
//     - 可选等 RTK
//     - 冻结 mission0：当前位置 + 当前 yaw
//     - mission0.x 为冻结时机头方向
//     - mission0.y 为左侧方向
//     - mission0.z 为世界竖直向上
//     - 全程 yaw 固定为 mission0.yaw0，使无人机始终正对墙面
//
// 默认轨迹：
//   1. 冻结 mission0：当前位置 + 当前 yaw
//   2. 起飞到 mission0.z + 1m
//   3. 沿机头方向前飞 1m
//   4. 向上飞 6m
//   5. 向右飞 3m
//   6. 向下飞 2m
//   7. 向左飞 3m
//   8. 向下飞 2m
//   9. 向右飞 3m
//  10. 悬停
//
// 坐标约定：
//   mission0.x：冻结时无人机机头方向
//   mission0.z：世界竖直向上
//   mission0.y：左侧方向
//   因此“无人机/人面向墙时的右边”是 mission0.y 的负方向。
//   lateral_direction = -1 表示先向右；+1 表示先向左。
//
// 轨迹平滑：
//   每个线段使用五次时间缩放：
//     s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5
//   使每段起点/终点速度和加速度均为 0。
//   command_speed 被解释为最大速度上限，而不是平均速度。
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

void publishPosVelAccSetpoint(
    ros::Publisher& pub,
    double px,
    double py,
    double pz,
    double vx,
    double vy,
    double vz,
    double ax,
    double ay,
    double az,
    double yaw)
{
    mavros_msgs::PositionTarget target;
    target.header.stamp = ros::Time::now();
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // 使用 position + velocity + acceleration + yaw
    // 只忽略 yaw_rate
    target.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    target.position.x = px;
    target.position.y = py;
    target.position.z = pz;

    target.velocity.x = vx;
    target.velocity.y = vy;
    target.velocity.z = vz;

    target.acceleration_or_force.x = ax;
    target.acceleration_or_force.y = ay;
    target.acceleration_or_force.z = az;

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

// ============================================================================
// 不使用 fabs(z) < 某阈值作为 EKF ready 判据。
// 原因：map 原点不一定等于当前起飞点，z 可能不是 0。
// 这里仅检查是否收到有效 local pose；真正稳定性由 mission0 freeze 阶段判断。
// ============================================================================
bool waitForPoseAndEKFReady(ros::Rate& rate, double timeout_s) {
    ROS_INFO("Waiting for local position data and basic pose validity...");
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
                std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
                std::isfinite(q.x) && std::isfinite(q.y) &&
                std::isfinite(q.z) && std::isfinite(q.w);

            const bool quat_ok =
                std::isfinite(q_norm) && q_norm > 0.7 && q_norm < 1.3;

            if (pose_finite && quat_ok) {
                double yaw = getYawFromQuat(q);
                ROS_INFO(
                    "Local pose ready: position=(%.2f, %.2f, %.2f), yaw=%.1f deg. "
                    "mission0 stability will be checked next.",
                    p.x, p.y, p.z, yaw * 180.0 / M_PI
                );
                return true;
            }

            ROS_WARN_THROTTLE(
                5.0,
                "Waiting for valid local pose: position=(%.2f, %.2f, %.2f), q_norm=%.3f",
                p.x, p.y, p.z, q_norm
            );
        } else {
            ROS_WARN_THROTTLE(5.0, "Waiting for /mavros/local_position/pose...");
        }

        if ((ros::Time::now() - start).toSec() > timeout_s) {
            if (g_pose_received) {
                ROS_ERROR(
                    "Local pose wait timeout! Current position=(%.2f, %.2f, %.2f)",
                    g_pose.pose.position.x,
                    g_pose.pose.position.y,
                    g_pose.pose.position.z
                );
            } else {
                ROS_ERROR("Local pose wait timeout: no /mavros/local_position/pose received");
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

// ============================================================================
// 五次时间缩放线段飞行
//
// 位置：
//   p(t) = p0 + s(tau) * (p1 - p0)
//
// 时间缩放：
//   s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5
//
// 其中：
//   tau = t / T
//   T = 1.875 * L / max_speed
//
// 这样设置时，五次曲线段内的最大速度不超过 max_speed。
// 代价是平均速度小于 max_speed，运动更平滑、更适合展示和近墙飞行。
// ============================================================================
bool flyQuinticSegment(
    ros::Publisher& setpoint_pub,
    ros::Rate& rate,
    const geometry_msgs::Point& start,
    const geometry_msgs::Point& goal,
    double yaw,
    double max_speed,
    double pos_tolerance,
    double extra_wait_s)
{
    const double dx = goal.x - start.x;
    const double dy = goal.y - start.y;
    const double dz = goal.z - start.z;

    const double L = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (L < 1e-6) {
        ROS_WARN("Quintic segment length near zero. Holding goal.");
        publishPositionSetpoint(setpoint_pub, goal.x, goal.y, goal.z, yaw);
        return true;
    }

    if (max_speed <= 0.0) {
        ROS_ERROR("Invalid max_speed: %.3f", max_speed);
        return false;
    }

    // 五次时间缩放 s'(tau) 的最大值为 1.875。
    // 若 T=L/v，则峰值速度会达到 1.875v。
    // 这里为了保证速度不超过 max_speed，使用 T=1.875L/max_speed。
    const double T = 1.875 * L / max_speed;

    ROS_INFO(
        "Quintic segment: start=(%.2f, %.2f, %.2f), goal=(%.2f, %.2f, %.2f), "
        "length=%.2f m, max_speed=%.2f m/s, duration=%.1f s",
        start.x, start.y, start.z,
        goal.x, goal.y, goal.z,
        L, max_speed, T
    );

    const ros::Time start_time = ros::Time::now();

    while (ros::ok()) {
        const double elapsed = (ros::Time::now() - start_time).toSec();

        double t = std::min(elapsed, T);
        double tau = t / T;

        const double tau2 = tau * tau;
        const double tau3 = tau2 * tau;
        const double tau4 = tau3 * tau;
        const double tau5 = tau4 * tau;

        const double s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;

        const double ds_dt =
            (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / T;

        const double d2s_dt2 =
            (60.0 * tau - 180.0 * tau2 + 120.0 * tau3) / (T * T);

        double px = start.x + s * dx;
        double py = start.y + s * dy;
        double pz = start.z + s * dz;

        double vx = ds_dt * dx;
        double vy = ds_dt * dy;
        double vz = ds_dt * dz;

        double ax = d2s_dt2 * dx;
        double ay = d2s_dt2 * dy;
        double az = d2s_dt2 * dz;

        if (elapsed >= T) {
            px = goal.x;
            py = goal.y;
            pz = goal.z;
            vx = vy = vz = 0.0;
            ax = ay = az = 0.0;
        }

        publishPosVelAccSetpoint(
            setpoint_pub,
            px, py, pz,
            vx, vy, vz,
            ax, ay, az,
            yaw
        );

        ros::spinOnce();

        const geometry_msgs::Point current = g_pose.pose.position;
        const double err = pointDistance(current, goal);

        ROS_INFO_THROTTLE(
            2.0,
            "Quintic progress: %.1f/%.1f s, cmd=(%.2f, %.2f, %.2f), "
            "vel=(%.2f, %.2f, %.2f), acc=(%.2f, %.2f, %.2f), "
            "current=(%.2f, %.2f, %.2f), err=%.2f",
            elapsed, T,
            px, py, pz,
            vx, vy, vz,
            ax, ay, az,
            current.x, current.y, current.z,
            err
        );

        if (elapsed >= T && err < pos_tolerance) {
            ROS_INFO(
                "Quintic segment reached: current=(%.2f, %.2f, %.2f), "
                "target=(%.2f, %.2f, %.2f), err=%.2f",
                current.x, current.y, current.z,
                goal.x, goal.y, goal.z,
                err
            );
            return true;
        }

        if (elapsed > T + extra_wait_s) {
            ROS_ERROR("Quintic segment timeout: target not reached. err=%.2f", err);
            return false;
        }

        rate.sleep();
    }

    return false;
}

std::vector<geometry_msgs::Point> buildHorizontalZigzagWaypoints(
    const MissionFrame& mission0,
    double takeoff_height,
    double approach_distance,
    double initial_climb,
    double horizontal_span,
    double vertical_step_down,
    int horizontal_segments,
    int first_lateral_direction)
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

    // 先向上飞 initial_climb
    z = takeoff_height + initial_climb;
    waypoints.push_back(
        missionLocalToMap(mission0, x, y, z)
    );

    // 水平 Zigzag：
    // 第 1 段：向右/左 horizontal_span
    // 然后下降 vertical_step_down
    // 第 2 段：反向 horizontal_span
    // 然后下降 vertical_step_down
    // 依次循环。
    int lateral_direction = first_lateral_direction;

    for (int seg = 0; seg < horizontal_segments; ++seg) {
        y += static_cast<double>(lateral_direction) * horizontal_span;

        waypoints.push_back(
            missionLocalToMap(mission0, x, y, z)
        );

        // 最后一条水平段之后不再强制下降。
        if (seg < horizontal_segments - 1) {
            z -= vertical_step_down;

            waypoints.push_back(
                missionLocalToMap(mission0, x, y, z)
            );
        }

        lateral_direction *= -1;
    }

    return waypoints;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_horizontal_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string mavros_ns;
    bool require_rtk;
    double rtk_wait_timeout;

    double takeoff_height;
    double approach_distance;
    double initial_climb;
    double horizontal_span;
    double vertical_step_down;
    int horizontal_segments;
    int first_lateral_direction;

    double command_speed;
    double corner_hold_s;
    double final_hold_s;
    bool hold_final_forever;

    double pos_tolerance;
    double segment_extra_wait_s;

    double connection_timeout_s;
    double ekf_timeout_s;

    int stable_frames_required;
    double pos_jitter_threshold;
    double yaw_jitter_threshold;
    double mission0_freeze_timeout_s;

    nh_private.param<std::string>("mavros_ns", mavros_ns, "/mavros");

    nh_private.param<bool>("require_rtk", require_rtk, false);
    nh_private.param<double>("rtk_wait_timeout", rtk_wait_timeout, 300.0);

    nh_private.param<double>("takeoff_height", takeoff_height, 1.0);
    nh_private.param<double>("approach_distance", approach_distance, 1.0);

    // 起飞并前飞后，先向上飞的高度，默认 6m
    nh_private.param<double>("initial_climb", initial_climb, 6.0);

    // 每条水平扫描线长度，默认 3m
    nh_private.param<double>("horizontal_span", horizontal_span, 3.0);

    // 每两条水平扫描线之间向下移动的高度，默认 2m
    nh_private.param<double>("vertical_step_down", vertical_step_down, 2.0);

    // 水平段数量。默认 3 段：右 3m、左 3m、右 3m
    nh_private.param<int>("horizontal_segments", horizontal_segments, 3);

    // -1 表示无人机/人面向墙时先向右；+1 表示先向左
    nh_private.param<int>("first_lateral_direction", first_lateral_direction, -1);

    // 解释为最大速度上限。五次时间缩放会自动拉长每段时间以保证不超过该速度。
    nh_private.param<double>("command_speed", command_speed, 0.2);

    nh_private.param<double>("corner_hold_s", corner_hold_s, 0.5);
    nh_private.param<double>("final_hold_s", final_hold_s, 10.0);
    nh_private.param<bool>("hold_final_forever", hold_final_forever, true);

    nh_private.param<double>("pos_tolerance", pos_tolerance, 0.25);
    nh_private.param<double>("segment_extra_wait_s", segment_extra_wait_s, 20.0);

    nh_private.param<double>("connection_timeout_s", connection_timeout_s, 30.0);
    nh_private.param<double>("ekf_timeout_s", ekf_timeout_s, 30.0);

    nh_private.param<int>("stable_frames_required", stable_frames_required, 20);
    nh_private.param<double>("pos_jitter_threshold", pos_jitter_threshold, 0.05);
    nh_private.param<double>("yaw_jitter_threshold", yaw_jitter_threshold, 0.05);
    nh_private.param<double>("mission0_freeze_timeout_s", mission0_freeze_timeout_s, 15.0);

    if (horizontal_segments <= 0) {
        ROS_ERROR("horizontal_segments must be > 0");
        return 1;
    }

    if (first_lateral_direction != -1 && first_lateral_direction != 1) {
        ROS_WARN("first_lateral_direction should be -1 or 1. Current=%d, forcing to -1",
                 first_lateral_direction);
        first_lateral_direction = -1;
    }

    if (initial_climb < 0.0 || horizontal_span < 0.0 || vertical_step_down < 0.0) {
        ROS_ERROR("initial_climb, horizontal_span, and vertical_step_down must be non-negative");
        return 1;
    }

    ROS_INFO("=== Wall Horizontal Zigzag Demo Configuration ===");
    ROS_INFO("mavros_ns: %s", mavros_ns.c_str());
    ROS_INFO("takeoff_height: %.2f m relative to mission0", takeoff_height);
    ROS_INFO("approach_distance: %.2f m forward", approach_distance);
    ROS_INFO("initial_climb: %.2f m", initial_climb);
    ROS_INFO("horizontal_span: %.2f m", horizontal_span);
    ROS_INFO("vertical_step_down: %.2f m", vertical_step_down);
    ROS_INFO("horizontal_segments: %d", horizontal_segments);
    ROS_INFO("first_lateral_direction: %d (-1=right, +1=left)", first_lateral_direction);
    ROS_INFO("command_speed: %.2f m/s as MAX speed limit", command_speed);
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

    // 2. local pose basic readiness
    if (!waitForPoseAndEKFReady(rate, ekf_timeout_s)) {
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
    std::vector<geometry_msgs::Point> waypoints = buildHorizontalZigzagWaypoints(
        mission0,
        takeoff_height,
        approach_distance,
        initial_climb,
        horizontal_span,
        vertical_step_down,
        horizontal_segments,
        first_lateral_direction
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

        ROS_INFO("Executing quintic segment %zu/%zu", i + 1, waypoints.size());

        if (!flyQuinticSegment(
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
        // 实际位置误差由 flyQuinticSegment 的 pos_tolerance 控制。
        current_start = goal;
    }

    const geometry_msgs::Point final_wp = waypoints.back();

    ROS_INFO("=== Wall horizontal zigzag demo complete. Final hold. ===");

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