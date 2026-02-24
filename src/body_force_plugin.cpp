/**
 * @brief Gazebo 机体系恒力插件 — 模拟喷枪后坐力等机体固连外力
 *
 * 以 AddRelativeForce 在机体坐标系施加恒定力。
 * 发布地面真值到 /gazebo/body_force_gt (geometry_msgs/Vector3Stamped, frame="body")。
 * 由 disturbance.launch 通过 ROS 参数服务器控制启停和力大小。
 *
 * 支持先启动 Gazebo 后设置参数的工作流（延迟检测 enabled 参数）。
 *
 * ROS 参数 (/body_force/ 命名空间):
 *   enabled:  是否启用（默认 false）
 *   force_x:  机体 X 方向力 N（默认 -5.0, 向后）
 *   force_y:  机体 Y 方向力 N（默认 0.0）
 *   force_z:  机体 Z 方向力 N（默认 0.0）
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace gazebo
{

class BodyForcePlugin : public ModelPlugin
{
public:
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
        model_ = model;

        // SDF 中只定义 link_name
        std::string link_name = "iris::base_link";
        if (sdf->HasElement("link_name"))
            link_name = sdf->Get<std::string>("link_name");

        link_ = model_->GetLink(link_name);
        if (!link_) {
            gzerr << "[BodyForcePlugin] Link '" << link_name << "' not found!\n";
            return;
        }

        // ROS 初始化
        if (!ros::isInitialized()) {
            int argc = 0;
            ros::init(argc, nullptr, "body_force_plugin", ros::init_options::NoSigintHandler);
        }

        nh_ = std::make_unique<ros::NodeHandle>();

        // 始终注册 OnUpdate 回调，延迟检测 enabled
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&BodyForcePlugin::OnUpdate, this));

        gzmsg << "[BodyForcePlugin] Loaded on link '" << link_name
              << "'. Waiting for /body_force/enabled param...\n";
    }

private:
    void OnUpdate()
    {
        if (!link_ || !nh_) return;

        // 每 1000 步检查一次参数服务器（支持运行时启停）
        check_count_++;
        if (check_count_ % 1000 == 0) {
            bool enabled = false;
            nh_->param<bool>("/body_force/enabled", enabled, false);
            if (enabled && !enabled_) {
                double fx, fy, fz;
                nh_->param<double>("/body_force/force_x", fx, -5.0);
                nh_->param<double>("/body_force/force_y", fy, 0.0);
                nh_->param<double>("/body_force/force_z", fz, 0.0);
                body_force_ = ignition::math::Vector3d(fx, fy, fz);
                if (!force_pub_) {
                    force_pub_ = nh_->advertise<geometry_msgs::Vector3Stamped>(
                        "/gazebo/body_force_gt", 10);
                }
                enabled_ = true;
                gzmsg << "[BodyForcePlugin] ENABLED! force=(" << body_force_ << ")\n";
            } else if (!enabled && enabled_) {
                enabled_ = false;
                gzmsg << "[BodyForcePlugin] DISABLED.\n";
            }
        }

        if (!enabled_) return;

        // 机体系恒力
        link_->AddRelativeForce(body_force_);

        // 发布地面真值（机体系）
        geometry_msgs::Vector3Stamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "body";
        msg.vector.x = body_force_.X();
        msg.vector.y = body_force_.Y();
        msg.vector.z = body_force_.Z();
        force_pub_.publish(msg);
    }

    physics::ModelPtr model_;
    physics::LinkPtr link_;
    event::ConnectionPtr update_connection_;

    bool enabled_ = false;
    int check_count_ = 0;
    ignition::math::Vector3d body_force_;

    std::unique_ptr<ros::NodeHandle> nh_;
    ros::Publisher force_pub_;
};

GZ_REGISTER_MODEL_PLUGIN(BodyForcePlugin)

}  // namespace gazebo
