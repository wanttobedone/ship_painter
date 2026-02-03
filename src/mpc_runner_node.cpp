/**
 * @file mpc_runner_node.cpp
 * @brief MPC-based trajectory tracking controller for Ship Painter UAV
 *
 * 功能说明:
 * - 替代trajectory_server_node，使用NMPC进行轨迹跟踪
 * - 输入: B-Spline轨迹、无人机状态
 * - 输出: 推力+角速度指令 (AttitudeTarget)
 *
 * 关键特性:
 * - 全姿态解算（Geometric Construction）：
 *   - 机体Z轴由物理决定（推力方向 = acc - g）
 *   - 机体X轴由任务决定（机头正对墙面法向量）
 *   - 通过叉乘构建正交坐标系，自然解决 Yaw 与 Roll/Pitch 的耦合
 * - MPC姿态闭环：Q_QUAT激活，统一优化位置和姿态
 * - 四元数半球一致性检查（防止 q/-q 跳变）
 * - 奇异点保护（自由落体、z_b平行x_des等情况）
 * - 求解失败时的安全悬停
 *
 * 升级说明 (2024):
 * - 引入几何构建法计算全姿态（从加速度+法向量）
 * - 移除外部Yaw PD控制器，由MPC统一优化
 * - 保持质量解耦设计（比力接口）
 *
 * @author Claude Code Assistant
 * @date 2024
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

// B-Spline相关
#include "ship_painter/Bspline.h"
#include "ship_painter/BsplineLayer.h"
#include "ship_painter/bspline.h"

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Acados生成的求解器头文件
// 注意: 这些文件由generate_mpc.py生成
extern "C" {
#include "acados_solver_ship_painter_mpc.h"
#include "acados_sim_solver_ship_painter_mpc.h"
}

#include <cmath>
#include <algorithm>

class MPCRunner {
public:
    MPCRunner();
    ~MPCRunner();

private:
    // ===== ROS接口 =====
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;           // 位置+姿态 (ENU世界坐标系)
    ros::Subscriber velocity_sub_;       // 速度 (ENU世界坐标系)
    ros::Subscriber bspline_sub_;
    ros::Publisher ctrl_pub_;
    ros::Publisher pred_traj_pub_;      // 预测轨迹可视化（参考轨迹，蓝色）
    ros::Publisher mpc_prediction_pub_; // MPC内部预测轨迹可视化（绿色）
    ros::Publisher ref_pose_pub_;       // 参考位姿可视化
    ros::Publisher actual_path_pub_;    // 实际飞行轨迹
    ros::Timer control_timer_;

    // 实际飞行轨迹存储
    nav_msgs::Path actual_path_;
    static constexpr int MAX_PATH_LENGTH_ = 2000;  // 最大轨迹点数

    // ===== Acados求解器 =====
    ship_painter_mpc_solver_capsule* acados_capsule_;
    bool solver_initialized_;

    // ===== MPC参数 (与generate_mpc.py保持一致!) =====
    static constexpr int NX_ = 10;      // 状态维度
    static constexpr int NU_ = 4;       // 控制维度
    static constexpr int N_HORIZON_ = 20;  // 预测步数
    static constexpr double T_HORIZON_ = 1.0;  // 预测时域
    static constexpr double DT_ = T_HORIZON_ / N_HORIZON_;  // 每步时长

    // 推力参数 (必须与generate_mpc.py一致!)
    double T_MAX_;
    double T_MIN_;
    double T_HOVER_;
    double MASS_;

    // ===== 状态变量 =====
    Eigen::Matrix<double, NX_, 1> current_state_;
    bool pose_received_;
    bool velocity_received_;

    // 轨迹
    std::vector<ship_painter::BSpline> layers_;
    std::vector<ship_painter::BSpline> transitions_;
    size_t current_layer_idx_;
    bool trajectory_active_;
    ros::Time trajectory_start_time_;
    std::vector<double> layer_end_times_;

    // Yaw奇异点保护
    double last_valid_yaw_;
    static constexpr double YAW_SINGULARITY_THRESHOLD_ = 0.1;  // 法向量xy分量阈值

    // ===== Yaw闭环控制参数 =====
    double yaw_kp_;           // Yaw比例增益
    double yaw_kd_;           // Yaw微分增益
    double last_yaw_error_;   // 上一次yaw误差（用于微分）
    double max_yaw_rate_;     // 最大yaw角速度限制

    // ===== 控制输出滤波 =====
    double filter_alpha_;     // 滤波系数 (0-1, 越小越平滑)
    double last_thrust_;      // 上一次推力
    double last_wx_, last_wy_, last_wz_;  // 上一次角速度

    // 统计信息
    int solve_success_count_;
    int solve_fail_count_;
    ros::Time last_status_print_;

    // 回调函数 =====
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void bsplineCallback(const ship_painter::BsplineLayer::ConstPtr& msg);
    void controlLoop(const ros::TimerEvent& event);

    // ===== 辅助函数 =====
    void loadParameters();
    bool initSolver();
    void publishHoverCommand();
    void publishPredictedTrajectory(const std::vector<Eigen::Vector3d>& positions);

    /**
     * @brief 从法向量计算Yaw角，带奇异点保护
     * @param normal 表面法向量
     * @return Yaw角 (rad)
     */
    double computeYawFromNormal(const Eigen::Vector3d& normal);

    /**
     * @brief 四元数半球一致性检查
     * @param q_ref 参考四元数
     * @param q_current 当前四元数
     * @return 调整后的参考四元数（与当前在同一半球）
     */
    Eigen::Quaterniond ensureQuaternionHemisphere(
        const Eigen::Quaterniond& q_ref,
        const Eigen::Quaterniond& q_current);

    /**
     * @brief 从四元数提取Yaw角
     * @param q 四元数
     * @return Yaw角 (rad)
     */
    double quaternionToYaw(const Eigen::Quaterniond& q);

    /**
     * @brief 计算Yaw角误差（处理±π跳变）
     * @param yaw_ref 参考Yaw角
     * @param yaw_current 当前Yaw角
     * @return Yaw误差 (rad), 范围[-π, π]
     */
    double computeYawError(double yaw_ref, double yaw_current);

    /**
     * @brief Yaw角PD控制器
     * @param yaw_ref 参考Yaw角
     * @param yaw_current 当前Yaw角
     * @param dt 时间步长
     * @return 计算的Yaw角速度 (rad/s)
     */
    double yawPDController(double yaw_ref, double yaw_current, double dt);

    /**
     * @brief 低通滤波器
     * @param new_val 新值
     * @param old_val 旧值
     * @param alpha 滤波系数
     * @return 滤波后的值
     */
    double lowPassFilter(double new_val, double old_val, double alpha);

    /**
     * @brief 全姿态解算器 (Differential Flatness + Geometric Construction)
     *
     * 核心思想：
     * 1. 机体Z轴由物理决定：必须指向合加速度方向（推力方向）
     * 2. 机体X轴由任务决定：机头正对墙面（法向量取反）
     * 3. 通过叉乘构建正交坐标系，自然解决 Yaw 与 Roll/Pitch 的耦合
     *
     * @param acc 轨迹加速度 (m/s²)
     * @param jerk 轨迹加加速度 (m/s³)
     * @param normal_vec 墙面法向量（指向墙外）
     * @param T_ref [out] 参考推力比力 (m/s²)
     * @param q_ref [out] 参考姿态四元数
     * @param omega_ref [out] 参考角速度 (rad/s)
     */
    void calculateDynamicReference(
        const Eigen::Vector3d& acc,
        const Eigen::Vector3d& jerk,
        const Eigen::Vector3d& normal_vec,
        double& T_ref,
        Eigen::Quaterniond& q_ref,
        Eigen::Vector3d& omega_ref);
};

// 构造函数
MPCRunner::MPCRunner()
    : nh_("~"),
      acados_capsule_(nullptr),
      solver_initialized_(false),
      pose_received_(false),
      velocity_received_(false),
      current_layer_idx_(0),
      trajectory_active_(false),
      last_valid_yaw_(0.0),
      last_yaw_error_(0.0),
      last_thrust_(0.0),
      last_wx_(0.0), last_wy_(0.0), last_wz_(0.0),
      solve_success_count_(0),
      solve_fail_count_(0)
{
    // 初始化状态为零（四元数w=1）
    current_state_.setZero();
    current_state_(3) = 1.0;  // qw = 1

    // 加载参数
    loadParameters();

    // 【关键修复】将滤波器状态初始化为悬停值，防止MPC接管时推力骤降
    // 原因：last_thrust_ 初始化为 0，导致第一帧滤波输出 = 0.6*9.81 + 0.4*0 = 5.9
    // 这会使飞机瞬间失去 40% 升力，触发掉高和电池故障保护
    last_thrust_ = T_HOVER_;

    // 初始化求解器
    if (!initSolver()) {
        ROS_FATAL("Failed to initialize Acados solver!");
        ros::shutdown();
        return;
    }

    // 获取mavros命名空间
    std::string mavros_ns;
    nh_.param<std::string>("mavros_ns", mavros_ns, "/mavros");

    // ===== 订阅者 =====
    // 位置+姿态 (ENU世界坐标系) - 与trajectory_server一致
    pose_sub_ = nh_.subscribe(mavros_ns + "/local_position/pose", 1,
        &MPCRunner::poseCallback, this);

    // 速度 (ENU世界坐标系) - 不是odom中的机体坐标系速度!
    velocity_sub_ = nh_.subscribe(mavros_ns + "/local_position/velocity_local", 1,
        &MPCRunner::velocityCallback, this);

    // B-Spline轨迹
    bspline_sub_ = nh_.subscribe("/planning/bspline_layers", 1,
        &MPCRunner::bsplineCallback, this);

    // ===== 发布者 =====
    // AttitudeTarget: 发送比力（加速度）+角速度 到推力映射节点
    // 注意：thrust 字段是加速度 (m/s²)，由 thrust_mapper_node 转换为油门
    ctrl_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
        "/ship_painter/mpc_raw_command", 1);

    // 预测轨迹可视化（参考轨迹，蓝色线）
    pred_traj_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/mpc/predicted_trajectory", 1);

    // MPC内部预测轨迹可视化（MPC求解器输出，绿色线）
    mpc_prediction_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/mpc/mpc_internal_prediction", 10);

    // 参考位姿可视化
    ref_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/mpc/reference_pose", 1);

    // 实际飞行轨迹
    actual_path_pub_ = nh_.advertise<nav_msgs::Path>(
        "/mpc/actual_path", 1);
    actual_path_.header.frame_id = "map";

    // ===== 控制定时器 =====
    // 50Hz控制频率 (与MPC步长匹配: dt = 0.05s = 50Hz)
    double control_rate;
    nh_.param<double>("control_rate", control_rate, 50.0);
    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / control_rate),
        &MPCRunner::controlLoop, this);

    last_status_print_ = ros::Time::now();

    ROS_INFO("MPC Runner Node Initialized (Full-State Feedforward Mode)");
    ROS_INFO("  MAVROS namespace: %s", mavros_ns.c_str());
    ROS_INFO("  Control rate: %.1f Hz", control_rate);
    ROS_INFO("  T_hover: %.2f m/s^2 (specific thrust)", T_HOVER_);
    ROS_INFO("  Thrust range: [%.1f, %.1f] m/s^2", T_MIN_, T_MAX_);
    ROS_INFO("  MPC horizon: %.2f s, %d steps", T_HORIZON_, N_HORIZON_);
    ROS_INFO("  Attitude control: MPC unified (Q_QUAT enabled)");
    ROS_INFO("  Feedforward: Dynamic thrust + attitude from differential flatness");
    ROS_INFO("  Output filter alpha: %.2f", filter_alpha_);
}

// ===== 析构函数 =====
MPCRunner::~MPCRunner() {
    if (acados_capsule_ != nullptr) {
        ship_painter_mpc_acados_free(acados_capsule_);
        ship_painter_mpc_acados_free_capsule(acados_capsule_);
    }
    ROS_INFO("MPC Runner shutdown. Solve stats: %d success, %d fail",
             solve_success_count_, solve_fail_count_);
}

// ===== 加载参数 =====
void MPCRunner::loadParameters() {
    // 物理参数 (必须与generate_mpc.py一致!)
    // 注意: 推力使用比力(specific thrust = 加速度 m/s²), 不是力(N)!
    // 这与rpg_mpc保持一致
    nh_.param<double>("mass", MASS_, 1.5);  // 仅用于日志显示
    nh_.param<double>("thrust_max", T_MAX_, 20.0);  // 最大比力 m/s² (≈2g)
    nh_.param<double>("thrust_min", T_MIN_, 2.0);   // 最小比力 m/s² (≈0.2g)
    T_HOVER_ = 9.8066;  // 悬停比力 = g (m/s²)

    // Yaw控制参数
    nh_.param<double>("yaw_kp", yaw_kp_, 2.0);          // Yaw比例增益
    nh_.param<double>("yaw_kd", yaw_kd_, 0.1);          // Yaw微分增益
    nh_.param<double>("max_yaw_rate", max_yaw_rate_, 1.5);  // 最大yaw角速度 (rad/s)

    // 控制输出滤波参数
    // alpha = 1.0 无滤波, alpha = 0.1 强滤波
    nh_.param<double>("filter_alpha", filter_alpha_, 0.5);
}

// ===== 初始化Acados求解器 =====
bool MPCRunner::initSolver() {
    acados_capsule_ = ship_painter_mpc_acados_create_capsule();
    if (acados_capsule_ == nullptr) {
        ROS_ERROR("Failed to create Acados capsule");
        return false;
    }

    int status = ship_painter_mpc_acados_create(acados_capsule_);
    if (status != 0) {
        ROS_ERROR("Failed to create Acados solver, status: %d", status);
        return false;
    }

    solver_initialized_ = true;
    ROS_INFO("Acados solver initialized successfully");
    return true;
}

// ===== Pose回调 (位置+姿态, ENU世界坐标系) =====
void MPCRunner::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // 位置 (ENU)
    current_state_(0) = msg->pose.position.x;
    current_state_(1) = msg->pose.position.y;
    current_state_(2) = msg->pose.position.z;

    // 四元数 (注意顺序: w, x, y, z)
    current_state_(3) = msg->pose.orientation.w;
    current_state_(4) = msg->pose.orientation.x;
    current_state_(5) = msg->pose.orientation.y;
    current_state_(6) = msg->pose.orientation.z;

    pose_received_ = true;
}

// ===== Velocity回调 (速度, ENU世界坐标系) =====
void MPCRunner::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    // 速度 (ENU世界坐标系, 不是机体坐标系!)
    current_state_(7) = msg->twist.linear.x;
    current_state_(8) = msg->twist.linear.y;
    current_state_(9) = msg->twist.linear.z;

    velocity_received_ = true;
}

// ===== B-Spline轨迹回调 =====
void MPCRunner::bsplineCallback(const ship_painter::BsplineLayer::ConstPtr& msg) {
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

        bspline.setDegree(layer_msg.order);
        if (!layer_msg.knots.empty()) {
            bspline.setKnots(layer_msg.knots);
        }
        bspline.setControlPoints(control_points);
        bspline.setNormals(normals);
        bspline.setTotalTime(layer_msg.duration);

        layers_.push_back(bspline);
    }

    layer_end_times_ = msg->layer_end_times;

    // 启动轨迹跟踪
    trajectory_active_ = true;
    trajectory_start_time_ = ros::Time::now();
    current_layer_idx_ = 0;

    ROS_INFO("MPC Runner: Loaded %zu trajectory layers", layers_.size());
}

// ===== 从法向量计算Yaw角（带奇异点保护）=====
double MPCRunner::computeYawFromNormal(const Eigen::Vector3d& normal) {
    // 喷涂方向 = 法向量取反（朝向表面）
    Eigen::Vector3d spray_dir = -normal;

    // 奇异点保护: 当法向量接近垂直时（水平甲板场景）
    // spray_dir.x() 和 spray_dir.y() 接近0，atan2不稳定
    if (std::abs(spray_dir.x()) < YAW_SINGULARITY_THRESHOLD_ &&
        std::abs(spray_dir.y()) < YAW_SINGULARITY_THRESHOLD_) {
        // 保持上一帧的yaw，防止机头乱转
        ROS_DEBUG_THROTTLE(1.0, "Yaw singularity detected, using last valid yaw: %.2f rad",
                          last_valid_yaw_);
        return last_valid_yaw_;
    }

    // 正常计算yaw
    double yaw = std::atan2(spray_dir.y(), spray_dir.x());
    last_valid_yaw_ = yaw;  // 更新有效值
    return yaw;
}

// ===== 四元数半球一致性检查 =====
Eigen::Quaterniond MPCRunner::ensureQuaternionHemisphere(
    const Eigen::Quaterniond& q_ref,
    const Eigen::Quaterniond& q_current) {

    // q 和 -q 表示相同的旋转
    // 如果点积 < 0，说明在不同半球，需要翻转
    if (q_ref.dot(q_current) < 0.0) {
        return Eigen::Quaterniond(-q_ref.w(), -q_ref.x(), -q_ref.y(), -q_ref.z());
    }
    return q_ref;
}

// ===== 从四元数提取Yaw角 =====
double MPCRunner::quaternionToYaw(const Eigen::Quaterniond& q) {
    // ZYX欧拉角分解，提取Yaw
    // yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    return std::atan2(siny_cosp, cosy_cosp);
}

// ===== 计算Yaw角误差（处理±π跳变）=====
double MPCRunner::computeYawError(double yaw_ref, double yaw_current) {
    double error = yaw_ref - yaw_current;

    // 规范化到 [-π, π]
    while (error > M_PI) error -= 2.0 * M_PI;
    while (error < -M_PI) error += 2.0 * M_PI;

    return error;
}

// ===== Yaw角PD控制器 =====
double MPCRunner::yawPDController(double yaw_ref, double yaw_current, double dt) {
    double yaw_error = computeYawError(yaw_ref, yaw_current);

    // P项
    double p_term = yaw_kp_ * yaw_error;

    // D项 (误差变化率)
    double d_term = 0.0;
    if (dt > 1e-6) {
        double error_rate = (yaw_error - last_yaw_error_) / dt;
        d_term = yaw_kd_ * error_rate;
    }

    // 更新上一次误差
    last_yaw_error_ = yaw_error;

    // 计算yaw_rate并限幅
    double yaw_rate = p_term + d_term;
    yaw_rate = std::clamp(yaw_rate, -max_yaw_rate_, max_yaw_rate_);

    return yaw_rate;
}

// ===== 低通滤波器 =====
double MPCRunner::lowPassFilter(double new_val, double old_val, double alpha) {
    return alpha * new_val + (1.0 - alpha) * old_val;
}

// ===== 全姿态解算器 (Geometric Construction) =====
void MPCRunner::calculateDynamicReference(
    const Eigen::Vector3d& acc,
    const Eigen::Vector3d& jerk,
    const Eigen::Vector3d& normal_vec,
    double& T_ref,
    Eigen::Quaterniond& q_ref,
    Eigen::Vector3d& omega_ref)
{
    // ===== 1. 计算合力向量（推力方向）=====
    // ENU坐标系：重力是 (0, 0, -9.8066)
    // 为了抵消重力并产生轨迹加速度：F_thrust = m * (a_traj - g)
    // 比力形式：a_thrust = a_traj - g = a_traj - (0, 0, -9.8066) = a_traj + (0, 0, 9.8066)
    const Eigen::Vector3d g(0.0, 0.0, -9.8066);
    Eigen::Vector3d t_vec = acc - g;  // 推力方向向量

    // 计算推力大小（比力）
    T_ref = t_vec.norm();

    // ===== 2. 构建机体 Z 轴 (z_b) =====
    // 安全检查：如果自由落体 (t_vec 接近 0)，保持 Z 轴向上
    if (T_ref < 1e-3) {
        T_ref = T_HOVER_;
        q_ref = Eigen::Quaterniond::Identity();
        omega_ref.setZero();
        return;
    }
    Eigen::Vector3d z_b = t_vec.normalized();

    // ===== 3. 确定期望的机头朝向 (x_des) =====
    // 喷涂任务：机头 (X轴) 指向墙面 => 法向量的反方向
    Eigen::Vector3d x_des = -normal_vec.normalized();

    // ===== 4. 构建机体 Y 轴 (y_b) =====
    // y_b = z_b × x_des（确保 Y 轴同时垂直于推力方向和期望朝向）
    Eigen::Vector3d y_b_temp = z_b.cross(x_des);

    // --- 奇异点保护 ---
    // 情况 A: z_b 和 x_des 平行（比如飞机垂直对着墙俯冲）
    if (y_b_temp.norm() < 1e-2) {
        // 备选方案：借用世界 Y 轴来生成
        y_b_temp = z_b.cross(Eigen::Vector3d::UnitY());
        if (y_b_temp.norm() < 1e-2) {
            // 极端情况：z_b 也和世界Y轴平行，用世界X轴
            y_b_temp = z_b.cross(Eigen::Vector3d::UnitX());
        }
    }
    Eigen::Vector3d y_b = y_b_temp.normalized();

    // ===== 5. 修正机体 X 轴 (x_b) =====
    // 确保 X, Y, Z 正交
    Eigen::Vector3d x_b = y_b.cross(z_b).normalized();

    // ===== 6. 构建旋转矩阵并转为四元数 =====
    Eigen::Matrix3d R_WB;
    R_WB.col(0) = x_b;
    R_WB.col(1) = y_b;
    R_WB.col(2) = z_b;

    q_ref = Eigen::Quaterniond(R_WB);
    q_ref.normalize();

    // ===== 7. 计算参考角速度（简化版本）=====
    // 从 jerk 近似计算 roll/pitch rate
    // 完整公式复杂，这里用简化方案
    if (T_ref > 0.1) {
        Eigen::Vector3d h_omega = jerk / T_ref;
        omega_ref.x() = -h_omega.dot(y_b);  // Roll rate
        omega_ref.y() = h_omega.dot(x_b);   // Pitch rate
        omega_ref.z() = 0.0;                // Yaw rate（由MPC优化）

        // 角速度限幅
        const double max_omega_xy = 2.0;  // rad/s
        omega_ref.x() = std::clamp(omega_ref.x(), -max_omega_xy, max_omega_xy);
        omega_ref.y() = std::clamp(omega_ref.y(), -max_omega_xy, max_omega_xy);
    } else {
        omega_ref.setZero();
    }
}

// ===== 主控制循环 =====
void MPCRunner::controlLoop(const ros::TimerEvent& event) {
    // 检查前置条件
    if (!solver_initialized_) {
        ROS_WARN_THROTTLE(5, "MPC solver not initialized");
        return;
    }

    if (!pose_received_) {
        ROS_WARN_THROTTLE(5, "Waiting for pose data...");
        return;
    }

    if (!velocity_received_) {
        ROS_WARN_THROTTLE(5, "Waiting for velocity data...");
        return;
    }

    if (!trajectory_active_ || layers_.empty()) {
        ROS_DEBUG_THROTTLE(5, "No active trajectory");
        return;
    }

    // 计算当前时间
    double t = (ros::Time::now() - trajectory_start_time_).toSec();

    // 检查层切换
    if (current_layer_idx_ < layers_.size()) {
        const auto& current_traj = layers_[current_layer_idx_];

        if (t > current_traj.getTotalTime()) {
            current_layer_idx_++;
            trajectory_start_time_ = ros::Time::now();
            t = 0.0;

            if (current_layer_idx_ >= layers_.size()) {
                ROS_INFO_ONCE("All layers completed! Hovering...");
                publishHoverCommand();
                return;
            }
            ROS_INFO("Switched to layer %zu/%zu", current_layer_idx_ + 1, layers_.size());
        }
    }

    const auto& traj = layers_[current_layer_idx_];
    double total_time = traj.getTotalTime();

    // 获取当前四元数用于半球检查
    Eigen::Quaterniond q_current(
        current_state_(3), current_state_(4),
        current_state_(5), current_state_(6));

    // ===== 设置MPC参考轨迹 =====
    std::vector<Eigen::Vector3d> pred_positions;  // 用于可视化
    // 在循环外定义并初始化 q_ref_last
    Eigen::Quaterniond q_ref_last = q_current;

    for (int i = 0; i <= N_HORIZON_; i++) {
        double t_pred = t + i * DT_;

        // 确保不超出轨迹范围
        if (t_pred > total_time) {
            t_pred = total_time;
        }

        // 获取参考状态
        Eigen::Vector3d p_ref = traj.getPosition(t_pred);
        Eigen::Vector3d v_ref = traj.getVelocity(t_pred);
        Eigen::Vector3d n_ref = traj.getNormal(t_pred);
        Eigen::Vector3d a_ref = traj.getAcceleration(t_pred);  // 轨迹加速度
        Eigen::Vector3d j_ref = traj.getJerk(t_pred);          // 轨迹加加速度

        pred_positions.push_back(p_ref);

        // ===== 几何构建法计算全姿态 =====
        // 输入：轨迹加速度 + 墙面法向量
        // 输出：动态推力、完整姿态四元数（含Roll/Pitch/Yaw）、角速度前馈
        double T_ref_dyn;
        Eigen::Quaterniond q_ref_dyn;
        Eigen::Vector3d omega_ref_dyn;
        calculateDynamicReference(a_ref, j_ref, n_ref, T_ref_dyn, q_ref_dyn, omega_ref_dyn);

        // 四元数半球一致性检查（防止 q 和 -q 跳变）
        // 第一步与当前状态比较，后续步与前一步比较
        Eigen::Quaterniond q_ref;
       if (i == 0) {
            // 第0步：跟当前真实姿态比
            q_ref = ensureQuaternionHemisphere(q_ref_dyn, q_current);
        } else {
            // 后续步：跟上一步的参考姿态比（确保预测序列内部连续）
            q_ref = ensureQuaternionHemisphere(q_ref_dyn, q_ref_last);
        }
        
        //  新 last 值给下一次循环用
        q_ref_last = q_ref;

        // 构建参考向量 y_ref = [p, q, v, u]
        double y_ref[NX_ + NU_];

        // 位置
        y_ref[0] = p_ref.x();
        y_ref[1] = p_ref.y();
        y_ref[2] = p_ref.z();

        // 四元数（含 Roll/Pitch 前馈倾角）
        y_ref[3] = q_ref.w();
        y_ref[4] = q_ref.x();
        y_ref[5] = q_ref.y();
        y_ref[6] = q_ref.z();

        // 速度
        y_ref[7] = v_ref.x();
        y_ref[8] = v_ref.y();
        y_ref[9] = v_ref.z();

        // 参考输入（动态前馈）
        y_ref[10] = T_ref_dyn;         // 动态推力（替换原来的 T_HOVER）
        y_ref[11] = omega_ref_dyn.x(); // 前馈 Roll rate
        y_ref[12] = omega_ref_dyn.y(); // 前馈 Pitch rate
        y_ref[13] = omega_ref_dyn.z(); // 前馈 Yaw rate

        // 设置到求解器
        if (i < N_HORIZON_) {
            ocp_nlp_cost_model_set(acados_capsule_->nlp_config,
                acados_capsule_->nlp_dims,
                acados_capsule_->nlp_in,
                i, "yref", y_ref);
        } else {
            // 终端参考（只有状态，无输入）
            ocp_nlp_cost_model_set(acados_capsule_->nlp_config,
                acados_capsule_->nlp_dims,
                acados_capsule_->nlp_in,
                i, "yref", y_ref);  // 终端会自动只取前NX_维
        }
    }

    // ===== 设置初始状态约束 =====
    ocp_nlp_constraints_model_set(acados_capsule_->nlp_config,
        acados_capsule_->nlp_dims,
        acados_capsule_->nlp_in,
        acados_capsule_->nlp_out,  // 新版Acados API需要nlp_out参数
        0, "lbx", current_state_.data());
    ocp_nlp_constraints_model_set(acados_capsule_->nlp_config,
        acados_capsule_->nlp_dims,
        acados_capsule_->nlp_in,
        acados_capsule_->nlp_out,  // 新版Acados API需要nlp_out参数
        0, "ubx", current_state_.data());

    // ===== 求解MPC =====
    int status = ship_painter_mpc_acados_solve(acados_capsule_);

    if (status != 0) {
        solve_fail_count_++;
        ROS_WARN_THROTTLE(1.0, "MPC solve failed! status=%d, publishing hover", status);
        publishHoverCommand();
        return;
    }

    solve_success_count_++;

    // ===== 提取并可视化 MPC 内部预测轨迹（绿色线）=====
    // 这是 MPC 求解器计算出的未来 N 步状态预测
    // 与参考轨迹（蓝色）对比，可以判断是"规划偏差"还是"动力不足"
    {
        visualization_msgs::Marker pred_line;
        pred_line.header.frame_id = "map";
        pred_line.header.stamp = ros::Time::now();
        pred_line.ns = "mpc_internal_prediction";
        pred_line.id = 0;
        pred_line.type = visualization_msgs::Marker::LINE_STRIP;
        pred_line.action = visualization_msgs::Marker::ADD;

        // 设置线宽
        pred_line.scale.x = 0.05;  // 5cm宽

        // 设置颜色：绿色（代表 MPC 自己的规划）
        pred_line.color.r = 0.0;
        pred_line.color.g = 1.0;
        pred_line.color.b = 0.0;
        pred_line.color.a = 0.8;

        // 遍历预测时域，提取每一步的状态
        for (int i = 0; i <= N_HORIZON_; i++) {
            double x_temp[NX_];  // 临时数组存状态，NX_=10

            // 【核心 API调用】从求解器内存中拿第 i 步的状态 "x"
            ocp_nlp_out_get(acados_capsule_->nlp_config,
                            acados_capsule_->nlp_dims,
                            acados_capsule_->nlp_out,
                            i, "x", x_temp);

            // 提取位置（前3维: px, py, pz）
            geometry_msgs::Point p;
            p.x = x_temp[0];
            p.y = x_temp[1];
            p.z = x_temp[2];

            pred_line.points.push_back(p);
        }

        // 发布出去
        mpc_prediction_pub_.publish(pred_line);
    }

    // ===== 提取控制量 u0 = [T, wx, wy, wz] =====
    double u0[NU_];
    ocp_nlp_out_get(acados_capsule_->nlp_config,
        acados_capsule_->nlp_dims,
        acados_capsule_->nlp_out,
        0, "u", u0);

    // ===== Yaw控制（已由MPC统一优化）=====
    // Q_QUAT已激活，MPC直接控制全姿态（含Roll/Pitch/Yaw），无需外挂PD控制器

    // 计算当前时刻的参考姿态（用于可视化和日志）
    Eigen::Vector3d a_now = traj.getAcceleration(t);
    Eigen::Vector3d j_now = traj.getJerk(t);
    Eigen::Vector3d n_now = traj.getNormal(t);
    double T_ref_now;
    Eigen::Quaterniond q_ref_now;
    Eigen::Vector3d omega_ref_now;
    calculateDynamicReference(a_now, j_now, n_now, T_ref_now, q_ref_now, omega_ref_now);

    // 提取当前 yaw 和目标 yaw 用于日志显示
    double current_yaw = quaternionToYaw(q_current);
    double target_yaw = quaternionToYaw(q_ref_now);

    // 直接使用MPC输出的yaw_rate（不再叠加PD控制）
    double wz_total = u0[3];

    // ===== 控制输出滤波（减少抖动）=====
    double thrust_filtered = lowPassFilter(u0[0], last_thrust_, filter_alpha_);
    double wx_filtered = lowPassFilter(u0[1], last_wx_, filter_alpha_);
    double wy_filtered = lowPassFilter(u0[2], last_wy_, filter_alpha_);
    double wz_filtered = lowPassFilter(wz_total, last_wz_, filter_alpha_);

    // 更新滤波器状态
    last_thrust_ = thrust_filtered;
    last_wx_ = wx_filtered;
    last_wy_ = wy_filtered;
    last_wz_ = wz_filtered;

    // ===== 发布控制指令（原始加速度，由 thrust_mapper 转换）=====
    mavros_msgs::AttitudeTarget cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "base_link";

    // type_mask: 忽略姿态，使用角速度
    // IGNORE_ROLL_RATE = 1, IGNORE_PITCH_RATE = 2, IGNORE_YAW_RATE = 4
    // IGNORE_THRUST = 64, IGNORE_ATTITUDE = 128
    cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;  // = 128

    // 直接输出比力（加速度 m/s²），由 thrust_mapper_node 转换为油门
    // 不再在此处做 sqrt 映射
    cmd.thrust = static_cast<float>(thrust_filtered);

    // 角速度（使用滤波后的值）
    cmd.body_rate.x = wx_filtered;
    cmd.body_rate.y = wy_filtered;
    cmd.body_rate.z = wz_filtered;

    ctrl_pub_.publish(cmd);

    // 发布预测轨迹可视化
    publishPredictedTrajectory(pred_positions);

    // 发布当前参考位姿（用于RViz可视化）
    // 使用几何构建法计算的全姿态（含Roll/Pitch/Yaw）
    {
        geometry_msgs::PoseStamped ref_pose_msg;
        ref_pose_msg.header.stamp = ros::Time::now();
        ref_pose_msg.header.frame_id = "map";

        Eigen::Vector3d p_ref_now = traj.getPosition(t);
        ref_pose_msg.pose.position.x = p_ref_now.x();
        ref_pose_msg.pose.position.y = p_ref_now.y();
        ref_pose_msg.pose.position.z = p_ref_now.z();

        // 使用几何构建法计算的全姿态四元数
        ref_pose_msg.pose.orientation.w = q_ref_now.w();
        ref_pose_msg.pose.orientation.x = q_ref_now.x();
        ref_pose_msg.pose.orientation.y = q_ref_now.y();
        ref_pose_msg.pose.orientation.z = q_ref_now.z();

        ref_pose_pub_.publish(ref_pose_msg);
    }

    // 发布实际飞行轨迹（黄色线条）
    {
        geometry_msgs::PoseStamped current_pose;
        current_pose.header.stamp = ros::Time::now();
        current_pose.header.frame_id = "map";
        current_pose.pose.position.x = current_state_(0);
        current_pose.pose.position.y = current_state_(1);
        current_pose.pose.position.z = current_state_(2);
        current_pose.pose.orientation.w = current_state_(3);
        current_pose.pose.orientation.x = current_state_(4);
        current_pose.pose.orientation.y = current_state_(5);
        current_pose.pose.orientation.z = current_state_(6);

        actual_path_.poses.push_back(current_pose);

        // 限制轨迹长度，防止内存无限增长
        if (actual_path_.poses.size() > MAX_PATH_LENGTH_) {
            actual_path_.poses.erase(actual_path_.poses.begin());
        }

        actual_path_.header.stamp = ros::Time::now();
        actual_path_pub_.publish(actual_path_);
    }

    // 定期打印状态
    if ((ros::Time::now() - last_status_print_).toSec() > 2.0) {
        Eigen::Vector3d p_ref = traj.getPosition(t);
        Eigen::Vector3d p_cur(current_state_(0), current_state_(1), current_state_(2));
        double pos_error = (p_ref - p_cur).norm();
        double yaw_error = computeYawError(target_yaw, current_yaw);

        ROS_INFO("[MPC] Layer %zu/%zu | t=%.1f/%.1f | pos_err=%.3fm | yaw_err=%.1f° | T=%.1f | wz=%.2f",
                 current_layer_idx_ + 1, layers_.size(),
                 t, total_time, pos_error, yaw_error * 180.0 / M_PI,
                 thrust_filtered, wz_filtered);
        last_status_print_ = ros::Time::now();
    }
}

// ===== 发布悬停指令 =====
void MPCRunner::publishHoverCommand() {
    mavros_msgs::AttitudeTarget cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

    // 悬停比力（加速度 = g），由 thrust_mapper 转换为油门
    cmd.thrust = static_cast<float>(T_HOVER_);

    // 零角速度
    cmd.body_rate.x = 0.0;
    cmd.body_rate.y = 0.0;
    cmd.body_rate.z = 0.0;

    ctrl_pub_.publish(cmd);
}

// ===== 发布预测轨迹可视化 =====
void MPCRunner::publishPredictedTrajectory(const std::vector<Eigen::Vector3d>& positions) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "mpc_prediction";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.03;  // 线宽

    // 蓝色表示预测
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 0.8;

    for (const auto& pos : positions) {
        geometry_msgs::Point p;
        p.x = pos.x();
        p.y = pos.y();
        p.z = pos.z();
        marker.points.push_back(p);
    }

    pred_traj_pub_.publish(marker);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_runner");

    try {
        MPCRunner runner;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("MPC Runner exception: %s", e.what());
        return 1;
    }

    return 0;
}
