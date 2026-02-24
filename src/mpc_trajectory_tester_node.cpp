/**
 * MPC 轨迹测试节点
 *
 * 自主完成起飞/解锁/OFFBOARD，然后生成参数化 B 样条轨迹
 * 发布给 mpc_runner_node 进行 MPC 闭环跟踪控制。
 *
 * Mode 0: 悬停（指定高度、朝向）
 * Mode 1: 画圆（指定半径、速度、圈数，鼻始终朝向圆心）
 * Mode 2: 偏航旋转（位置不动，yaw 往返旋转，用于解耦世界系/机体系）
 * Mode 3: 八字飞行（Lissajous 曲线，提供高频激励）
 * Mode 4: 椭圆（泛化画圆，鼻朝圆心）
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "ship_painter/Bspline.h"
#include "ship_painter/BsplineLayer.h"
#include "ship_painter/bspline.h"

#include <Eigen/Dense>
#include <cmath>
#include <vector>

// ===== 状态机 =====
enum class FlightState {
    WAIT_CONNECTION,
    PREFLIGHT,
    ARM_OFFBOARD,
    TAKEOFF,
    STABILIZE,
    PUBLISH_TRAJECTORY,
    MPC_ACTIVE
};

// ===== 全局状态 =====
mavros_msgs::State g_fcu_state;
geometry_msgs::PoseStamped g_current_pose;
bool g_pose_received = false;

void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    g_fcu_state = *msg;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    g_current_pose = *msg;
    g_pose_received = true;
}

// ===== 轨迹构建 =====

/**
 * Mode 0: 悬停轨迹
 * 10 个相同控制点，手动 clamped 均匀 knots
 */
ship_painter::BsplineLayer buildHoverTrajectory(double height, double yaw) {
    const int degree = 3;
    const int num_points = 10;
    const double duration = 3600.0;  // 1 小时

    ship_painter::Bspline bspline_msg;
    bspline_msg.order = degree;
    bspline_msg.start_time = ros::Time::now();
    bspline_msg.duration = duration;

    // 控制点：全部相同
    for (int i = 0; i < num_points; i++) {
        geometry_msgs::Point pt;
        pt.x = 0.0;
        pt.y = 0.0;
        pt.z = height;
        bspline_msg.pos_pts.push_back(pt);

        // 法向量：x_des = -normal, 鼻朝向 (cos(yaw), sin(yaw), 0)
        geometry_msgs::Vector3 n;
        n.x = -std::cos(yaw);
        n.y = -std::sin(yaw);
        n.z = 0.0;
        bspline_msg.normals.push_back(n);
    }

    // Clamped 均匀 knots: m = n + degree + 1 = 14
    int m = num_points + degree + 1;
    int n_interior = num_points - degree - 1;  // = 6
    for (int i = 0; i <= degree; i++) {
        bspline_msg.knots.push_back(0.0);
    }
    for (int i = 1; i <= n_interior; i++) {
        bspline_msg.knots.push_back(duration * i / (n_interior + 1));
    }
    for (int i = 0; i <= degree; i++) {
        bspline_msg.knots.push_back(duration);
    }

    // 校验 knots 数量
    ROS_ASSERT_MSG(static_cast<int>(bspline_msg.knots.size()) == m,
                   "Hover knots count mismatch: %zu vs %d",
                   bspline_msg.knots.size(), m);

    ship_painter::BsplineLayer layer_msg;
    layer_msg.layers.push_back(bspline_msg);
    layer_msg.layer_end_times.push_back(duration);

    ROS_INFO("Hover trajectory: %d pts, duration=%.0fs, yaw=%.1f deg",
             num_points, duration, yaw * 180.0 / M_PI);

    return layer_msg;
}

/**
 * Mode 1: 圆形轨迹（鼻始终朝向圆心）
 * 圆心 (-R, 0)，起点 (0, 0, H) 在 theta=0
 * 使用 BSpline::fitFromContour 自动生成弦长参数化 knots
 */
ship_painter::BsplineLayer buildCircleTrajectory(double radius, double height,
                                                  double speed, int laps) {
    const int pts_per_lap = 72;  // 每 5° 一个点
    const int total_points = pts_per_lap * laps + 1;  // +1 闭合最后一圈
    const double d_theta = 2.0 * M_PI / pts_per_lap;

    double cx = -radius;  // 圆心 X（使起点在 x=0）
    double cy = 0.0;      // 圆心 Y

    std::vector<Eigen::Vector3d> contour_points;
    std::vector<Eigen::Vector3d> contour_normals;

    for (int i = 0; i < total_points; i++) {
        double theta = i * d_theta;

        // 位置：圆上的点
        double x = cx + radius * std::cos(theta);
        double y = cy + radius * std::sin(theta);
        contour_points.emplace_back(x, y, height);

        // 法向量：使鼻朝向圆心
        // 鼻方向 = (cx - x, cy - y, 0).normalized() = (-cos(theta), -sin(theta), 0)
        // x_des = -normal => normal = (cos(theta), sin(theta), 0)
        contour_normals.emplace_back(std::cos(theta), std::sin(theta), 0.0);
    }

    // 使用 BSpline 库自动生成 knots
    ship_painter::BSpline bspline;
    bspline.setDegree(3);
    bspline.fitFromContour(contour_points, contour_normals, speed, false);

    // 提取到消息
    ship_painter::Bspline bspline_msg;
    bspline_msg.order = bspline.getDegree();
    bspline_msg.start_time = ros::Time::now();
    bspline_msg.duration = bspline.getTotalTime();
    bspline_msg.knots = bspline.getKnots();

    for (const auto& pt : bspline.getControlPoints()) {
        geometry_msgs::Point p;
        p.x = pt.x(); p.y = pt.y(); p.z = pt.z();
        bspline_msg.pos_pts.push_back(p);
    }
    for (const auto& n : bspline.getNormals()) {
        geometry_msgs::Vector3 vn;
        vn.x = n.x(); vn.y = n.y(); vn.z = n.z();
        bspline_msg.normals.push_back(vn);
    }

    ship_painter::BsplineLayer layer_msg;
    layer_msg.layers.push_back(bspline_msg);
    layer_msg.layer_end_times.push_back(bspline.getTotalTime());

    double expected_time = 2.0 * M_PI * radius * laps / speed;
    ROS_INFO("Circle trajectory: %d pts, R=%.1fm, v=%.1fm/s, laps=%d, duration=%.1fs (expected %.1fs)",
             total_points, radius, speed, laps, bspline.getTotalTime(), expected_time);

    return layer_msg;
}

/**
 * Mode 2: 偏航旋转轨迹（位置不动，yaw 0→2π*laps→0 往返）
 * 解耦世界系与机体系的关键数据：世界系风不变，机体系喷枪力在世界系下画圆
 * 位置全部固定，法向量旋转。使用手动 clamped knots（fitFromContour 弦长为 0 不可用）
 */
ship_painter::BsplineLayer buildYawSpinTrajectory(double height,
                                                   double yaw_speed, int laps) {
    const int degree = 3;
    const int pts_per_rotation = 72;  // 5° 分辨率
    // 往返：0→2π*laps（前半）→0（后半）
    const int total_points = 2 * pts_per_rotation * laps + 1;
    const double total_angle = 2.0 * 2.0 * M_PI * laps;
    const double duration = total_angle / yaw_speed;

    ship_painter::Bspline bspline_msg;
    bspline_msg.order = degree;
    bspline_msg.start_time = ros::Time::now();
    bspline_msg.duration = duration;

    for (int i = 0; i < total_points; i++) {
        // 位置：固定
        geometry_msgs::Point pt;
        pt.x = 0.0; pt.y = 0.0; pt.z = height;
        bspline_msg.pos_pts.push_back(pt);

        // Yaw 角：三角波 (0 → 2π*laps → 0)
        double frac = static_cast<double>(i) / (total_points - 1);
        double yaw;
        if (frac <= 0.5) {
            yaw = frac * 2.0 * 2.0 * M_PI * laps;  // 正转
        } else {
            yaw = (1.0 - frac) * 2.0 * 2.0 * M_PI * laps;  // 反转
        }

        // 法向量：鼻朝 (cos(yaw), sin(yaw)) → normal = (-cos(yaw), -sin(yaw), 0)
        geometry_msgs::Vector3 n;
        n.x = -std::cos(yaw);
        n.y = -std::sin(yaw);
        n.z = 0.0;
        bspline_msg.normals.push_back(n);
    }

    // Clamped 均匀 knots
    int m = total_points + degree + 1;
    int n_interior = total_points - degree - 1;
    for (int i = 0; i <= degree; i++)
        bspline_msg.knots.push_back(0.0);
    for (int i = 1; i <= n_interior; i++)
        bspline_msg.knots.push_back(duration * i / (n_interior + 1));
    for (int i = 0; i <= degree; i++)
        bspline_msg.knots.push_back(duration);

    ROS_ASSERT_MSG(static_cast<int>(bspline_msg.knots.size()) == m,
                   "YawSpin knots count mismatch: %zu vs %d",
                   bspline_msg.knots.size(), m);

    ship_painter::BsplineLayer layer_msg;
    layer_msg.layers.push_back(bspline_msg);
    layer_msg.layer_end_times.push_back(duration);

    ROS_INFO("YawSpin trajectory: %d pts, speed=%.1f rad/s, laps=%d, duration=%.1fs",
             total_points, yaw_speed, laps, duration);

    return layer_msg;
}

/**
 * Mode 3: 八字飞行轨迹（Lissajous 曲线）
 * x = A*sin(θ), y = A/2*sin(2θ)
 * 提供激进的俯仰/滚转变化，用于训练瞬变分支
 */
ship_painter::BsplineLayer buildFigure8Trajectory(double scale, double height,
                                                    double speed, int laps) {
    const int pts_per_lap = 72;
    const int total_points = pts_per_lap * laps + 1;
    const double d_theta = 2.0 * M_PI / pts_per_lap;

    double A = scale;
    double B = scale * 0.5;

    std::vector<Eigen::Vector3d> contour_points;
    std::vector<Eigen::Vector3d> contour_normals;

    for (int i = 0; i < total_points; i++) {
        double theta = i * d_theta;

        double x = A * std::sin(theta);
        double y = B * std::sin(2.0 * theta);
        contour_points.emplace_back(x, y, height);

        // 法向量：鼻朝向原点
        // 鼻方向 = (-x, -y, 0).normalized(), x_des = -normal → normal = (x, y, 0).normalized()
        double r = std::sqrt(x * x + y * y);
        if (r > 1e-3) {
            contour_normals.emplace_back(x / r, y / r, 0.0);
        } else {
            // 在原点交叉处，用切线的法线方向
            double dx = A * std::cos(theta);
            double dy = 2.0 * B * std::cos(2.0 * theta);
            double dn = std::sqrt(dx * dx + dy * dy);
            contour_normals.emplace_back(-dy / dn, dx / dn, 0.0);
        }
    }

    ship_painter::BSpline bspline;
    bspline.setDegree(3);
    bspline.fitFromContour(contour_points, contour_normals, speed, false);

    // 提取到消息（与 circle 相同模式）
    ship_painter::Bspline bspline_msg;
    bspline_msg.order = bspline.getDegree();
    bspline_msg.start_time = ros::Time::now();
    bspline_msg.duration = bspline.getTotalTime();
    bspline_msg.knots = bspline.getKnots();

    for (const auto& pt : bspline.getControlPoints()) {
        geometry_msgs::Point p;
        p.x = pt.x(); p.y = pt.y(); p.z = pt.z();
        bspline_msg.pos_pts.push_back(p);
    }
    for (const auto& n : bspline.getNormals()) {
        geometry_msgs::Vector3 vn;
        vn.x = n.x(); vn.y = n.y(); vn.z = n.z();
        bspline_msg.normals.push_back(vn);
    }

    ship_painter::BsplineLayer layer_msg;
    layer_msg.layers.push_back(bspline_msg);
    layer_msg.layer_end_times.push_back(bspline.getTotalTime());

    ROS_INFO("Figure8 trajectory: %d pts, scale=%.1fm, v=%.1fm/s, laps=%d, duration=%.1fs",
             total_points, scale, speed, laps, bspline.getTotalTime());

    return layer_msg;
}

/**
 * Mode 4: 椭圆轨迹（鼻朝向椭圆中心）
 * 圆的泛化：a=b 时退化为圆
 * center=(-a, 0)，起点 (0, 0, H) 在 theta=0
 */
ship_painter::BsplineLayer buildEllipseTrajectory(double a, double b, double height,
                                                    double speed, int laps) {
    const int pts_per_lap = 72;
    const int total_points = pts_per_lap * laps + 1;
    const double d_theta = 2.0 * M_PI / pts_per_lap;

    double cx = -a;
    double cy = 0.0;

    std::vector<Eigen::Vector3d> contour_points;
    std::vector<Eigen::Vector3d> contour_normals;

    for (int i = 0; i < total_points; i++) {
        double theta = i * d_theta;

        double x = cx + a * std::cos(theta);
        double y = cy + b * std::sin(theta);
        contour_points.emplace_back(x, y, height);

        // 法向量：鼻朝向椭圆中心
        double dx = cx - x;
        double dy = cy - y;
        double dn = std::sqrt(dx * dx + dy * dy);
        contour_normals.emplace_back(-dx / dn, -dy / dn, 0.0);
    }

    ship_painter::BSpline bspline;
    bspline.setDegree(3);
    bspline.fitFromContour(contour_points, contour_normals, speed, false);

    ship_painter::Bspline bspline_msg;
    bspline_msg.order = bspline.getDegree();
    bspline_msg.start_time = ros::Time::now();
    bspline_msg.duration = bspline.getTotalTime();
    bspline_msg.knots = bspline.getKnots();

    for (const auto& pt : bspline.getControlPoints()) {
        geometry_msgs::Point p;
        p.x = pt.x(); p.y = pt.y(); p.z = pt.z();
        bspline_msg.pos_pts.push_back(p);
    }
    for (const auto& n : bspline.getNormals()) {
        geometry_msgs::Vector3 vn;
        vn.x = n.x(); vn.y = n.y(); vn.z = n.z();
        bspline_msg.normals.push_back(vn);
    }

    ship_painter::BsplineLayer layer_msg;
    layer_msg.layers.push_back(bspline_msg);
    layer_msg.layer_end_times.push_back(bspline.getTotalTime());

    ROS_INFO("Ellipse trajectory: %d pts, a=%.1fm, b=%.1fm, v=%.1fm/s, laps=%d, duration=%.1fs",
             total_points, a, b, speed, laps, bspline.getTotalTime());

    return layer_msg;
}

// ===== 主函数 =====
int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_trajectory_tester_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // --- 加载参数 ---
    int test_mode;
    double hover_height, hover_yaw;
    double circle_radius, circle_speed;
    int circle_laps;
    double yaw_spin_speed;
    int yaw_spin_laps;
    double figure8_scale, figure8_speed;
    int figure8_laps;
    double ellipse_a, ellipse_b, ellipse_speed;
    int ellipse_laps;
    double stabilize_duration;
    std::string mavros_ns;

    pnh.param<int>("test_mode", test_mode, 0);
    pnh.param<double>("hover_height", hover_height, 2.0);
    pnh.param<double>("hover_yaw", hover_yaw, 0.0);
    pnh.param<double>("circle_radius", circle_radius, 3.0);
    pnh.param<double>("circle_speed", circle_speed, 0.5);
    pnh.param<int>("circle_laps", circle_laps, 2);
    pnh.param<double>("yaw_spin_speed", yaw_spin_speed, 0.5);
    pnh.param<int>("yaw_spin_laps", yaw_spin_laps, 2);
    pnh.param<double>("figure8_scale", figure8_scale, 2.0);
    pnh.param<double>("figure8_speed", figure8_speed, 0.5);
    pnh.param<int>("figure8_laps", figure8_laps, 2);
    pnh.param<double>("ellipse_a", ellipse_a, 3.0);
    pnh.param<double>("ellipse_b", ellipse_b, 2.0);
    pnh.param<double>("ellipse_speed", ellipse_speed, 0.5);
    pnh.param<int>("ellipse_laps", ellipse_laps, 2);
    pnh.param<double>("stabilize_duration", stabilize_duration, 3.0);
    pnh.param<std::string>("mavros_ns", mavros_ns, "/mavros");

    const char* mode_names[] = {"HOVER", "CIRCLE", "YAW_SPIN", "FIGURE8", "ELLIPSE"};
    const char* mode_str = (test_mode >= 0 && test_mode <= 4) ? mode_names[test_mode] : "UNKNOWN";

    ROS_INFO("=== MPC Trajectory Tester ===");
    ROS_INFO("  Mode: %s (%d)", mode_str, test_mode);
    ROS_INFO("  Height: %.1f m", hover_height);
    if (test_mode == 0) {
        ROS_INFO("  Yaw: %.1f deg", hover_yaw * 180.0 / M_PI);
    } else if (test_mode == 1) {
        ROS_INFO("  Radius: %.1f m, Speed: %.1f m/s, Laps: %d",
                 circle_radius, circle_speed, circle_laps);
    } else if (test_mode == 2) {
        ROS_INFO("  Yaw speed: %.1f rad/s, Laps: %d", yaw_spin_speed, yaw_spin_laps);
    } else if (test_mode == 3) {
        ROS_INFO("  Scale: %.1f m, Speed: %.1f m/s, Laps: %d",
                 figure8_scale, figure8_speed, figure8_laps);
    } else if (test_mode == 4) {
        ROS_INFO("  a: %.1f m, b: %.1f m, Speed: %.1f m/s, Laps: %d",
                 ellipse_a, ellipse_b, ellipse_speed, ellipse_laps);
    }

    // --- ROS 接口 ---
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        mavros_ns + "/state", 10, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        mavros_ns + "/local_position/pose", 10, poseCallback);

    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>(
        mavros_ns + "/setpoint_position/local", 10);
    ros::Publisher bspline_pub = nh.advertise<ship_painter::BsplineLayer>(
        "/planning/bspline_layers", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
        mavros_ns + "/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        mavros_ns + "/set_mode");

    ros::Rate rate(20.0);

    // --- 起飞目标位姿 ---
    // 画圆/椭圆模式：起飞时 yaw 朝向圆心 (π rad = 面朝 -X)
    double initial_yaw;
    if (test_mode == 1 || test_mode == 4) {
        initial_yaw = M_PI;  // 朝圆心
    } else {
        initial_yaw = hover_yaw;  // hover, yaw_spin, figure8
    }

    geometry_msgs::PoseStamped takeoff_pose;
    takeoff_pose.header.frame_id = "map";
    takeoff_pose.pose.position.x = 0.0;
    takeoff_pose.pose.position.y = 0.0;
    takeoff_pose.pose.position.z = hover_height;
    takeoff_pose.pose.orientation.w = std::cos(initial_yaw / 2.0);
    takeoff_pose.pose.orientation.x = 0.0;
    takeoff_pose.pose.orientation.y = 0.0;
    takeoff_pose.pose.orientation.z = std::sin(initial_yaw / 2.0);

    // --- 状态机 ---
    FlightState state = FlightState::WAIT_CONNECTION;
    int preflight_count = 0;
    ros::Time last_request;
    ros::Time stabilize_start;

    while (ros::ok()) {
        switch (state) {

        case FlightState::WAIT_CONNECTION:
            if (g_fcu_state.connected) {
                ROS_INFO("FCU connected");
                state = FlightState::PREFLIGHT;
                preflight_count = 100;
            } else {
                ROS_INFO_THROTTLE(2, "Waiting for FCU connection...");
            }
            break;

        case FlightState::PREFLIGHT:
            takeoff_pose.header.stamp = ros::Time::now();
            setpoint_pub.publish(takeoff_pose);
            preflight_count--;
            if (preflight_count <= 0) {
                ROS_INFO("Preflight setpoints sent, switching to OFFBOARD");
                state = FlightState::ARM_OFFBOARD;
                last_request = ros::Time::now();
            }
            break;

        case FlightState::ARM_OFFBOARD:
            takeoff_pose.header.stamp = ros::Time::now();
            setpoint_pub.publish(takeoff_pose);

            if (g_fcu_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                mavros_msgs::SetMode offb_mode;
                offb_mode.request.custom_mode = "OFFBOARD";
                if (set_mode_client.call(offb_mode) && offb_mode.response.mode_sent) {
                    ROS_INFO("OFFBOARD mode enabled");
                }
                last_request = ros::Time::now();
            } else if (!g_fcu_state.armed &&
                       (ros::Time::now() - last_request > ros::Duration(5.0))) {
                mavros_msgs::CommandBool arm_cmd;
                arm_cmd.request.value = true;
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }

            if (g_fcu_state.armed && g_fcu_state.mode == "OFFBOARD") {
                ROS_INFO("Armed in OFFBOARD, taking off to %.1f m", hover_height);
                state = FlightState::TAKEOFF;
            }
            break;

        case FlightState::TAKEOFF:
            takeoff_pose.header.stamp = ros::Time::now();
            setpoint_pub.publish(takeoff_pose);

            if (g_pose_received &&
                std::abs(g_current_pose.pose.position.z - hover_height) < 0.3) {
                ROS_INFO("Reached target height, stabilizing for %.1f s", stabilize_duration);
                stabilize_start = ros::Time::now();
                state = FlightState::STABILIZE;
            } else {
                ROS_INFO_THROTTLE(2, "Climbing... z=%.2f / %.2f",
                                  g_current_pose.pose.position.z, hover_height);
            }
            break;

        case FlightState::STABILIZE:
            takeoff_pose.header.stamp = ros::Time::now();
            setpoint_pub.publish(takeoff_pose);

            if ((ros::Time::now() - stabilize_start).toSec() >= stabilize_duration) {
                ROS_INFO("Stabilization complete, publishing trajectory to MPC");
                state = FlightState::PUBLISH_TRAJECTORY;
            }
            break;

        case FlightState::PUBLISH_TRAJECTORY: {
            ship_painter::BsplineLayer traj_msg;
            if (test_mode == 0) {
                traj_msg = buildHoverTrajectory(hover_height, hover_yaw);
            } else if (test_mode == 1) {
                traj_msg = buildCircleTrajectory(circle_radius, hover_height,
                                                  circle_speed, circle_laps);
            } else if (test_mode == 2) {
                traj_msg = buildYawSpinTrajectory(hover_height, yaw_spin_speed,
                                                   yaw_spin_laps);
            } else if (test_mode == 3) {
                traj_msg = buildFigure8Trajectory(figure8_scale, hover_height,
                                                    figure8_speed, figure8_laps);
            } else if (test_mode == 4) {
                traj_msg = buildEllipseTrajectory(ellipse_a, ellipse_b, hover_height,
                                                    ellipse_speed, ellipse_laps);
            } else {
                ROS_ERROR("Unknown test_mode: %d, falling back to hover", test_mode);
                traj_msg = buildHoverTrajectory(hover_height, hover_yaw);
            }
            bspline_pub.publish(traj_msg);

            ROS_INFO("Trajectory published to /planning/bspline_layers, MPC takeover");
            state = FlightState::MPC_ACTIVE;
            break;
        }

        case FlightState::MPC_ACTIVE:
            // MPC + thrust_mapper 已接管，不再发布 PoseStamped
            ROS_INFO_THROTTLE(10, "MPC active | z=%.2f",
                              g_current_pose.pose.position.z);
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
