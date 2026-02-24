#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

/**
 * @brief Momentum-based Disturbance Observer (MDOB)
 *
 * 基于动量守恒的扰动力观测器。通过比较"预期动量"与"实测动量"的差异，
 * 实时解算无人机受到的合外扰动力（世界系 ENU + 机体系）。
 *
 * 算法要点：
 * 1. 一阶执行器滞后模型：对推力指令进行低通滤波，模拟电机物理延迟
 * 2. 动量积分器：显式欧拉法积分预期动量
 * 3. 观测器反馈：Ko 增益矩阵驱动预期动量向实测动量收敛
 *
 * dt 不做硬编码，而是在 update() 中根据实时时间戳动态计算。
 */
class MDOBEstimator {
public:
    /**
     * @param mass  无人机质量 (kg)
     * @param tau   执行器一阶滞后时间常数 (s)，推荐 0.04~0.06
     * @param Ko    观测器对角增益向量，推荐 (5.0, 5.0, 5.0)
     */
    MDOBEstimator(double mass, double tau, const Eigen::Vector3d& Ko);

    /**
     * @brief 每帧调用的核心更新函数
     *
     * @param current_time      当前时间戳 (s)，用于计算实际 dt
     * @param v_world_actual    世界系 ENU 速度 (m/s)
     * @param q_actual          机体姿态四元数 (w,x,y,z)
     * @param thrust_cmd_accel  MPC 原始推力输出（比力 m/s^2），不是经过软件滤波的值
     */
    void update(double current_time,
                const Eigen::Vector3d& v_world_actual,
                const Eigen::Quaterniond& q_actual,
                double thrust_cmd_accel);

    /** @brief 获取世界系 (ENU) 合外扰动力 (N) */
    Eigen::Vector3d getDisturbanceWorld() const;

    /** @brief 获取机体系 (Body) 合外扰动力 (N) */
    Eigen::Vector3d getDisturbanceBody() const;

    /** @brief 观测器是否已完成初始化（至少执行过一次 update） */
    bool isInitialized() const { return initialized_; }

private:
    // 物理参数
    double mass_;
    double tau_;
    Eigen::Vector3d Ko_;

    // 内部状态
    Eigen::Vector3d p_hat_;           // 预期动量 (kg*m/s)
    Eigen::Vector3d f_dist_world_;    // 世界系扰动力观测值 (N)
    Eigen::Vector3d f_dist_body_;     // 机体系扰动力观测值 (N)
    double t_real_;                   // 执行器滞后后的实际推力 (N)

    // 时间管理
    double last_time_;
    bool initialized_;

    // dt 安全钳位范围 (s)
    static constexpr double DT_MIN_ = 0.001;
    static constexpr double DT_MAX_ = 0.1;
};
