/**
 * @brief Gazebo 风扰动插件 — 基于 Ornstein-Uhlenbeck 过程
 *
 * 运作方式:
 *   插件以 ModelPlugin 挂在 SDF 模型上（始终加载），
 *   始终注册 OnUpdate 回调，在回调中动态检测 /wind/enabled 参数。
 *   支持先启动 Gazebo 后设置参数的工作流。
 *
 * Launch 用法:
 *   # 无风（默认）
 *   roslaunch ship_painter mpc_test.launch
 *
 *   # 开风
 *   roslaunch ship_painter mpc_test.launch wind_enabled:=true
 *
 * ROS 参数（/wind/ 命名空间）:
 *   enabled:      是否施加风力（默认 false）
 *   theta:        OU 回归速率（默认 0.5）
 *   mu_x/y/z:     均值风力 N（默认 0）
 *   sigma_x/y/z:  波动幅度 N（默认 1.0, 1.0, 0.5）
 *   force_max:    单轴力上限 N（默认 5.0）
 *
 * ROS Topic:
 *   发布: /gazebo/wind_force_gt  (geometry_msgs/Vector3Stamped)  地面真值
 *   订阅: /gazebo/wind_mean_cmd  (geometry_msgs/Vector3Stamped)  运行时动态修改均值
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <random>
#include <algorithm>
#include <mutex>

namespace gazebo
{

class WorldWindPlugin : public ModelPlugin
{
public:
    WorldWindPlugin()
        : enabled_(false),
          params_loaded_(false),
          gen_(std::random_device{}()),
          dist_(0.0, 1.0)
    {
    }

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
        model_ = model;

        // SDF 只保留 link_name
        std::string link_name = "iris::base_link";
        if (sdf->HasElement("link_name"))
            link_name = sdf->Get<std::string>("link_name");

        link_ = model_->GetLink(link_name);
        if (!link_) {
            gzerr << "[WorldWindPlugin] Link '" << link_name << "' not found!\n";
            return;
        }

        // ROS 初始化
        if (!ros::isInitialized()) {
            int argc = 0;
            ros::init(argc, nullptr, "world_wind_plugin", ros::init_options::NoSigintHandler);
        }

        nh_ = std::make_unique<ros::NodeHandle>();

        // 始终注册 OnUpdate 回调，在回调中检测 enabled 状态
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&WorldWindPlugin::OnUpdate, this));

        gzmsg << "[WorldWindPlugin] Loaded on link '" << link_name
              << "'. Waiting for /wind/enabled param...\n";
    }

private:
    void OnUpdate()
    {
        if (!link_ || !nh_) return;

        // 每 1000 步检查一次参数服务器（支持运行时启停）
        check_count_++;
        if (check_count_ % 1000 == 0) {
            bool enabled = false;
            nh_->param<bool>("/wind/enabled", enabled, false);
            if (enabled && !enabled_) {
                enabled_ = true;
                loadParams();
            } else if (!enabled && enabled_) {
                enabled_ = false;
                gzmsg << "[WorldWindPlugin] Wind DISABLED.\n";
            }
        }

        if (!enabled_) return;

        // === 风力已启用，执行 OU 过程 ===

        double dt = model_->GetWorld()->Physics()->GetMaxStepSize();
        double sqrt_dt = std::sqrt(dt);

        ignition::math::Vector3d mu_local;
        {
            std::lock_guard<std::mutex> lock(mu_mutex_);
            mu_local = mu_;
        }

        // OU 过程: Euler-Maruyama 离散化
        force_ou_.X() += theta_ * (mu_local.X() - force_ou_.X()) * dt
                       + sigma_.X() * sqrt_dt * dist_(gen_);
        force_ou_.Y() += theta_ * (mu_local.Y() - force_ou_.Y()) * dt
                       + sigma_.Y() * sqrt_dt * dist_(gen_);
        force_ou_.Z() += theta_ * (mu_local.Z() - force_ou_.Z()) * dt
                       + sigma_.Z() * sqrt_dt * dist_(gen_);

        // 限幅
        force_ou_.X() = std::clamp(force_ou_.X(), -force_max_, force_max_);
        force_ou_.Y() = std::clamp(force_ou_.Y(), -force_max_, force_max_);
        force_ou_.Z() = std::clamp(force_ou_.Z(), -force_max_, force_max_);

        // 施加力（世界系）
        link_->AddForce(force_ou_);

        // 发布地面真值
        geometry_msgs::Vector3Stamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        msg.vector.x = force_ou_.X();
        msg.vector.y = force_ou_.Y();
        msg.vector.z = force_ou_.Z();
        wind_pub_.publish(msg);
    }

    void loadParams()
    {
        nh_->param<double>("/wind/theta", theta_, 0.5);

        double mx, my, mz;
        nh_->param<double>("/wind/mu_x", mx, 0.0);
        nh_->param<double>("/wind/mu_y", my, 0.0);
        nh_->param<double>("/wind/mu_z", mz, 0.0);
        mu_ = ignition::math::Vector3d(mx, my, mz);

        double sx, sy, sz;
        nh_->param<double>("/wind/sigma_x", sx, 1.0);
        nh_->param<double>("/wind/sigma_y", sy, 1.0);
        nh_->param<double>("/wind/sigma_z", sz, 0.5);
        sigma_ = ignition::math::Vector3d(sx, sy, sz);

        nh_->param<double>("/wind/force_max", force_max_, 5.0);

        force_ou_ = mu_;

        wind_pub_ = nh_->advertise<geometry_msgs::Vector3Stamped>("/gazebo/wind_force_gt", 10);
        mean_sub_ = nh_->subscribe("/gazebo/wind_mean_cmd", 1,
                                   &WorldWindPlugin::meanCallback, this);

        gzmsg << "[WorldWindPlugin] Wind ENABLED!\n"
              << "  theta=" << theta_
              << " mu=(" << mu_ << ") sigma=(" << sigma_ << ")\n"
              << "  force_max=" << force_max_ << "\n";
    }

    void meanCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mu_mutex_);
        mu_.X() = msg->vector.x;
        mu_.Y() = msg->vector.y;
        mu_.Z() = msg->vector.z;
        gzmsg << "[WorldWindPlugin] Mean updated to (" << mu_ << ")\n";
    }

    // Gazebo
    physics::ModelPtr model_;
    physics::LinkPtr link_;
    event::ConnectionPtr update_connection_;

    // 开关
    bool enabled_;
    bool params_loaded_;
    int check_count_ = 0;

    // OU 过程参数
    double theta_ = 0.5;
    ignition::math::Vector3d mu_;
    ignition::math::Vector3d sigma_;
    ignition::math::Vector3d force_ou_;
    double force_max_ = 5.0;

    // 随机数
    std::mt19937 gen_;
    std::normal_distribution<double> dist_;

    // ROS
    std::unique_ptr<ros::NodeHandle> nh_;
    ros::Publisher wind_pub_;
    ros::Subscriber mean_sub_;
    std::mutex mu_mutex_;
};

GZ_REGISTER_MODEL_PLUGIN(WorldWindPlugin)

}  // namespace gazebo
