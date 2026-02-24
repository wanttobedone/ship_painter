//MDBO，动量观测器，用于观测残差力,纯数学计算节点

#include "ship_painter/mdob_estimator.h"
#include <algorithm>

MDOBEstimator::MDOBEstimator(double mass, double tau, const Eigen::Vector3d& Ko)
    : mass_(mass),
      tau_(tau),
      Ko_(Ko),
      p_hat_(Eigen::Vector3d::Zero()),
      f_dist_world_(Eigen::Vector3d::Zero()),
      f_dist_body_(Eigen::Vector3d::Zero()),
      t_real_(mass * 9.8066),  // 初始化为悬停推力 (N)
      last_time_(0.0),
      initialized_(false)
{
}

void MDOBEstimator::update(double current_time,
                           const Eigen::Vector3d& v_world_actual,
                           const Eigen::Quaterniond& q_actual,
                           double thrust_cmd_accel)
{
    // 首次调用：初始化内部状态，不产生观测输出
    if (!initialized_) {
        p_hat_ = mass_ * v_world_actual;  // 预期动量 = 实测动量
        t_real_ = mass_ * thrust_cmd_accel;  // 推力初始化 (N)
        last_time_ = current_time;
        initialized_ = true;
        return;
    }

    // 动态计算 dt
    double dt = current_time - last_time_;
    dt = std::clamp(dt, DT_MIN_, DT_MAX_);
    last_time_ = current_time;

    //   1. 执行器一阶滞后模型  
    // 将比力 (m/s^2) 转换为物理推力 (N)
    double thrust_cmd_N = mass_ * thrust_cmd_accel;
    double alpha = dt / (tau_ + dt);
    t_real_ = alpha * thrust_cmd_N + (1.0 - alpha) * t_real_;

    //   2. 计算预期动量导数  
    // ENU 坐标系：重力方向为 -Z
    const Eigen::Vector3d gravity(0.0, 0.0, -9.8066);

    // 推力在机体 Z 轴方向，转换到世界系
    Eigen::Matrix3d R_wb = q_actual.toRotationMatrix();
    Eigen::Vector3d thrust_world = R_wb * Eigen::Vector3d(0.0, 0.0, t_real_);

    // 预期动量导数 = 重力 + 推力 + 扰动力（上一帧的观测值作为反馈）
    Eigen::Vector3d p_hat_dot = mass_ * gravity + thrust_world + f_dist_world_;

    //   3. 动量积分（显式欧拉法） 
    p_hat_ += p_hat_dot * dt;

    //   4. 扰动力观测值更新  
    // 实测动量
    Eigen::Vector3d p_actual = mass_ * v_world_actual;

    // 观测器反馈：Ko * (实测动量 - 预期动量)
    f_dist_world_ = Ko_.asDiagonal() * (p_actual - p_hat_);

    //   5. 转换到机体系  
    f_dist_body_ = q_actual.inverse() * f_dist_world_;
}

Eigen::Vector3d MDOBEstimator::getDisturbanceWorld() const {
    return f_dist_world_;
}

Eigen::Vector3d MDOBEstimator::getDisturbanceBody() const {
    return f_dist_body_;
}
