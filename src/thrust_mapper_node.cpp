/**
 * @file thrust_mapper_node.cpp
 * @brief 推力映射节点：将 MPC 输出的加速度转换为飞控油门
 *
 * 功能说明:
 * - 订阅 MPC 节点发布的原始命令（比力/加速度）
 * - 将加速度转换为物理力：F = m * a
 * - 将物理力映射为油门：throttle = sqrt(F / F_max)
 * - 发布给飞控执行
 *
 * 设计理念:
 * - MPC 只思考"加速度"，不需要知道飞机质量
 * - 推力映射与控制计算解耦，换飞机只需改参数
 *
 * @author Claude Code Assistant
 * @date 2024
 */

#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <algorithm>
#include <cmath>

class ThrustMapper {
public:
    ThrustMapper() : nh_("~") {
        // 加载参数
        nh_.param<double>("mass", mass_, 1.5);              // 飞机质量 (kg)
        nh_.param<double>("max_thrust_n", max_force_, 28.27); // 电机最大拉力 (N)
        nh_.param<double>("min_throttle", min_throttle_, 0.05); // 最小油门（防止电机停转）
        nh_.param<double>("max_throttle", max_throttle_, 0.95); // 最大油门（留安全余量）

        // 计算悬停油门（用于日志）
        double hover_force = mass_ * 9.8066;
        double hover_throttle = std::sqrt(hover_force / max_force_);

        // 订阅 MPC 原始命令
        cmd_sub_ = nh_.subscribe("mpc_cmd", 1, &ThrustMapper::commandCallback, this);

        // 发布给飞控
        std::string mavros_ns;
        nh_.param<std::string>("mavros_ns", mavros_ns, "/mavros");
        cmd_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
            mavros_ns + "/setpoint_raw/attitude", 1);

        ROS_INFO("Thrust Mapper Node Initialized");
        ROS_INFO("  Mass: %.2f kg", mass_);
        ROS_INFO("  Max thrust: %.2f N", max_force_);
        ROS_INFO("  Hover throttle: %.3f (at g=9.81)", hover_throttle);
        ROS_INFO("  Throttle range: [%.2f, %.2f]", min_throttle_, max_throttle_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_;
    ros::Publisher cmd_pub_;

    double mass_;           // 飞机质量 (kg)
    double max_force_;      // 电机最大拉力 (N)
    double min_throttle_;   // 最小油门
    double max_throttle_;   // 最大油门

    // 统计信息
    int cmd_count_ = 0;
    ros::Time last_log_time_;

    void commandCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
        mavros_msgs::AttitudeTarget out_msg = *msg;  // 复制角速度等数据

        // 1. 获取 MPC 发来的期望加速度 (单位 m/s^2)
        // 约定：上游 msg.thrust 字段装的是加速度（比力），不是油门
        double desired_accel = msg->thrust;

        // 2. 转换为物理力 (F = ma)
        double desired_force = desired_accel * mass_;

        // 3. 计算推力比 (Force Ratio)
        double force_ratio = desired_force / max_force_;

        // 4. 映射为油门 (非线性映射: 油门 = sqrt(力比例))
        // 电机推力与油门通常是平方关系：F ∝ throttle²
        double throttle = std::sqrt(std::max(0.0, force_ratio));

        // 5. 限幅
        throttle = std::clamp(throttle, min_throttle_, max_throttle_);
        out_msg.thrust = static_cast<float>(throttle);

        // 6. 发布给飞控
        cmd_pub_.publish(out_msg);

        // 定期日志
        cmd_count_++;
        if (last_log_time_.isZero()) {
            last_log_time_ = ros::Time::now();
        }
        if ((ros::Time::now() - last_log_time_).toSec() > 5.0) {
            ROS_INFO("[ThrustMapper] accel=%.2f m/s^2 -> force=%.2f N -> throttle=%.3f | count=%d",
                     desired_accel, desired_force, throttle, cmd_count_);
            last_log_time_ = ros::Time::now();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "thrust_mapper");

    try {
        ThrustMapper mapper;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Thrust Mapper exception: %s", e.what());
        return 1;
    }

    return 0;
}
