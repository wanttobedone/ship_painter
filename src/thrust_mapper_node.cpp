/**
 * @brief 推力映射节点：将 MPC 输出的加速度转换为飞控油门
 *
 * 功能说明:
 * - 订阅 MPC 节点发布的原始命令（比力/加速度）
 * - 将加速度转换为物理力：F = m * a
 * - 将物理力映射为油门：throttle = sqrt(F / F_max)
 * - 发布给飞控执行
 *
 * - MPC 只思考"加速度"，不需要知道飞机质量
 * - 推力映射与控制计算解耦，换飞机只需改参数
 */

#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <algorithm>
#include <cmath>

class ThrustMapper {
public:
    ThrustMapper() : nh_private_("~") { // 1. 初始化私有句柄，专门用于读取参数
        // 使用 nh_private_ 读取参数
        // 这样 launch 文件里的 <param> 才能被读到
        nh_private_.param<double>("mass", mass_, 1.5);
        nh_private_.param<double>("max_thrust_n", max_force_, 28.27);
        nh_private_.param<double>("min_throttle", min_throttle_, 0.05);
        nh_private_.param<double>("max_throttle", max_throttle_, 0.95);

        // 计算悬停油门（用于日志）
        double hover_force = mass_ * 9.8066;
        double hover_throttle = std::sqrt(hover_force / max_force_);

        // 2. 【关键修改】创建公有句柄，专门用于订阅和发布话题
        ros::NodeHandle nh_public; 

        // 使用公有句柄订阅 "mpc_cmd"
        // 这样 launch 文件里的 <remap from="mpc_cmd" ...> 就能正常生效了
        cmd_sub_ = nh_public.subscribe("mpc_cmd", 1, &ThrustMapper::commandCallback, this);

        // 获取 MAVROS 命名空间 (参数还是用私有句柄读)
        std::string mavros_ns;
        nh_private_.param<std::string>("mavros_ns", mavros_ns, "/mavros");
        
        // 发布话题也建议使用公有句柄
        cmd_pub_ = nh_public.advertise<mavros_msgs::AttitudeTarget>(
            mavros_ns + "/setpoint_raw/attitude", 1);

        ROS_INFO("Thrust Mapper Node Initialized");
        ROS_INFO("  Mass: %.2f kg", mass_);
        ROS_INFO("  Max thrust: %.2f N", max_force_);
        ROS_INFO("  Hover throttle: %.3f (at g=9.81)", hover_throttle);
        ROS_INFO("  Throttle range: [%.2f, %.2f]", min_throttle_, max_throttle_);
    }

private:
    ros::NodeHandle nh_private_; // 私有句柄成员变量
    ros::Subscriber cmd_sub_;
    ros::Publisher cmd_pub_;

    double mass_;
    double max_force_;
    double min_throttle_;
    double max_throttle_;

    int cmd_count_ = 0;
    ros::Time last_log_time_;

    void commandCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
        mavros_msgs::AttitudeTarget out_msg = *msg;

        // 1. 获取加速度 (m/s^2)
        double desired_accel = msg->thrust;

        // 2. 转换为力 (F = ma)
        double desired_force = desired_accel * mass_;

        // 3. 计算力比例
        double force_ratio = desired_force / max_force_;

        // 4. 映射为油门 (throttle = sqrt(ratio))
        double throttle = std::sqrt(std::max(0.0, force_ratio));

        // 5. 安全限幅
        throttle = std::clamp(throttle, min_throttle_, max_throttle_);
        out_msg.thrust = static_cast<float>(throttle);

        // 6. 发布
        cmd_pub_.publish(out_msg);

        // 日志
        cmd_count_++;
        if (last_log_time_.isZero()) last_log_time_ = ros::Time::now();
        if ((ros::Time::now() - last_log_time_).toSec() > 5.0) {
            ROS_INFO("[ThrustMapper] accel=%.2f -> throttle=%.3f | count=%d",
                     desired_accel, throttle, cmd_count_);
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
