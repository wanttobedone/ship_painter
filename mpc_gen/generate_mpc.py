#!/usr/bin/env python3
"""
Ship Painter MPC Solver Generator (Fixed Version)
=================================================
修正说明:
1. 物理参数匹配 iris.sdf (Mass=1.5kg, MaxForce=28.27N) -> T_MAX ≈ 19.0 m/s^2
2. 调整权重以消除剧烈俯仰震荡 (Q_POS降低, R_INPUT大幅提高)
"""

from acados_template import AcadosOcp, AcadosModel, AcadosOcpSolver
from casadi import SX, vertcat
import numpy as np
from scipy.linalg import block_diag
import os


# =============================================================================
# 参数设置 (已针对 PX4 SITL Iris 模型优化)
# =============================================================================

# 物理参数
# 注意: 推力使用比力(specific thrust = 加速度 m/s²), 不是力(N)!
G_VAL = 9.8066          # 重力加速度 (m/s^2)

# 推力限制 (比力, m/s²)
# 基于 iris.sdf 计算: 
#   Max Force = 28.27 N
#   Mass = 1.5 kg
#   Max Accel = 28.27 / 1.5 = 18.846 m/s²
T_MAX = 19.0            # 最大推力比力 (留一点余量取19.0)
T_MIN = 1.0             # 最小推力比力 (防止电机停转)
T_HOVER = G_VAL         # 悬停推力比力 = g ≈ 9.81 m/s²

# 角速度限制
# 限制稍微收紧一点，防止求解器生成过大的角速度指令
RATE_MAX_XY = 2.5       # 最大Roll/Pitch角速度 (rad/s)
RATE_MAX_Z = 1.5        # 最大Yaw角速度 (rad/s)

# MPC时域配置
N_HORIZON = 20          # 预测步数
T_HORIZON = 1.0         # 预测时域 (秒)
# 每步时长: dt = 0.05s

# -----------------------------------------------------------------------------
# 代价函数权重 (关键修改)
# -----------------------------------------------------------------------------

# 位置权重 [x, y, z]
# 原来是 [100, 100, 200]，太大了，导致飞机为了修正1cm误差而猛打杆
# 修改为: 降低权重，容忍少量误差，换取平稳
Q_POS = [20.0, 20.0, 50.0]

# 四元数权重 [qw, qx, qy, qz]
# 激活姿态控制：MPC 将主动跟踪参考姿态（含 Roll/Pitch 前馈倾角和 Yaw）
Q_QUAT = [50.0, 50.0, 50.0, 50.0]

# 速度权重 [vx, vy, vz]
Q_VEL = [10.0, 10.0, 10.0]

# 输入权重 [Thrust, wx, wy, wz]
# 原来是 [0.1, 1.0, 1.0, 0.5]
# 修改为: 大幅增加惩罚，抑制高频震荡和剧烈动作
# wx, wy 设为 10.0，强迫飞机动作“温柔”
R_INPUT = [0.5, 10.0, 10.0, 5.0]


def export_drone_model():
    """定义无人机刚体动力学模型 (无需修改)"""
    model_name = 'ship_painter_mpc'

    # 状态变量
    px = SX.sym('px'); py = SX.sym('py'); pz = SX.sym('pz')
    qw = SX.sym('qw'); qx = SX.sym('qx'); qy = SX.sym('qy'); qz = SX.sym('qz')
    vx = SX.sym('vx'); vy = SX.sym('vy'); vz = SX.sym('vz')
    x = vertcat(px, py, pz, qw, qx, qy, qz, vx, vy, vz)

    # 控制变量
    T = SX.sym('T')      # 比力 (m/s^2)
    wx = SX.sym('wx'); wy = SX.sym('wy'); wz = SX.sym('wz')
    u = vertcat(T, wx, wy, wz)

    # 动力学方程
    p_dot = vertcat(vx, vy, vz)

    # 旋转矩阵 R_WB
    R_WB = vertcat(
        1 - 2*(qy**2 + qz**2),  2*(qx*qy - qw*qz),      2*(qx*qz + qw*qy),
        2*(qx*qy + qw*qz),      1 - 2*(qx**2 + qz**2),  2*(qy*qz - qw*qx),
        2*(qx*qz - qw*qy),      2*(qy*qz + qw*qx),      1 - 2*(qx**2 + qy**2)
    ).reshape((3, 3))

    # 速度导数 (T是加速度)
    v_dot = vertcat(
        2 * (qw * qy + qx * qz) * T,           
        2 * (qy * qz - qw * qx) * T,           
        (1 - 2*qx*qx - 2*qy*qy) * T - G_VAL    
    )

    # 四元数导数
    q_dot = 0.5 * vertcat(
        -qx*wx - qy*wy - qz*wz,
         qw*wx + qy*wz - qz*wy,
         qw*wy - qx*wz + qz*wx,
         qw*wz + qx*wy - qy*wx
    )

    x_dot = vertcat(p_dot, q_dot, v_dot)

    model = AcadosModel()
    model.f_expl_expr = x_dot
    model.x = x
    model.u = u
    model.name = model_name
    return model


def setup_ocp():
    """配置最优控制问题 (OCP)"""
    ocp = AcadosOcp()
    model = export_drone_model()
    ocp.model = model

    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    ocp.dims.N = N_HORIZON
    ocp.solver_options.tf = T_HORIZON
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.model.cost_y_expr = vertcat(model.x, model.u)
    ocp.model.cost_y_expr_e = model.x

    # 权重矩阵
    Q_diag = np.array(Q_POS + Q_QUAT + Q_VEL)
    R_diag = np.array(R_INPUT)
    Q_mat = np.diag(Q_diag)
    R_mat = np.diag(R_diag)
    ocp.cost.W = block_diag(Q_mat, R_mat)
    ocp.cost.W_e = Q_mat

    # 初始参考
    y_ref = np.zeros(ny)
    y_ref[3] = 1.0
    y_ref[10] = T_HOVER
    ocp.cost.yref = y_ref
    ocp.cost.yref_e = y_ref[:nx]

    # 约束
    ocp.constraints.lbu = np.array([T_MIN, -RATE_MAX_XY, -RATE_MAX_XY, -RATE_MAX_Z])
    ocp.constraints.ubu = np.array([T_MAX, RATE_MAX_XY, RATE_MAX_XY, RATE_MAX_Z])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])

    x0 = np.zeros(nx); x0[3] = 1.0
    ocp.constraints.x0 = x0

    # 求解器选项
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.nlp_solver_max_iter = 1
    ocp.solver_options.print_level = 0

    return ocp


def main():
    print("=" * 60)
    print("Ship Painter MPC Solver Generator (Optimized)")
    print("=" * 60)

    print(f"\n[配置参数]")
    print(f"  重力加速度: {G_VAL} m/s²")
    print(f"  比力范围: [{T_MIN}, {T_MAX}] m/s² (Target: Iris 1.5kg)")
    
    print(f"\n[代价权重 - 已优化防震荡]")
    print(f"  位置 Q_pos: {Q_POS}")
    print(f"  输入 R_input: {R_INPUT}")

    ocp = setup_ocp()
    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'c_generated_code')
    
    # 这里的 json_file 路径可能需要根据您的环境微调
    solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')

    print("\n" + "=" * 60)
    print("生成成功! 请务必执行以下后续步骤:")
    print("1. 重新编译 C++ 代码: cp -r c_generated_code ... && catkin_make")
    print("2. 检查 mpc_runner_node.cpp 中的推力映射是否使用了 sqrt()")
    print("3. 检查 launch 文件中的 thrust_max 是否设为了 28.27 (Max Force) 或 19.0 (Max Accel)")
    print("   (取决于你的 C++ 节点是除以 Force 还是 Accel)")
    print("=" * 60)

    return solver


if __name__ == '__main__':
    main()