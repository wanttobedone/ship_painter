#!/usr/bin/env python3
"""
Ship Painter MPC Solver Generator
=================================
生成基于Acados的NMPC求解器，用于无人机轨迹跟踪控制。

功能特点:
- 10维状态空间: [px, py, pz, qw, qx, qy, qz, vx, vy, vz]
- 4维控制输入: [Thrust, wx, wy, wz]
- 包含完整的姿态(Yaw)跟踪
- 使用NONLINEAR_LS代价函数（适合四元数）
- 使用ENU坐标系（与ROS/MAVROS一致）

坐标系约定:
- ENU (East-North-Up): X向东, Y向北, Z向上
- 重力: [0, 0, -9.81] (向下)
- 推力: [0, 0, +T] 在机体坐标系中向上

使用方法:
1. 根据实际无人机修改下方参数区
2. 运行: python3 generate_mpc.py
3. 将c_generated_code复制到ship_painter/src/mpc/

"""

from acados_template import AcadosOcp, AcadosModel, AcadosOcpSolver
from casadi import SX, vertcat, mtimes
import numpy as np
from scipy.linalg import block_diag
import os


# 参数设置根据实际无人机修改这里

# 物理参数
# 注意: 推力使用比力(specific thrust = 加速度 m/s²), 不是力(N)!
# 这与rpg_mpc保持一致
G_VAL = 9.8066          # 重力加速度 (m/s^2)

# 推力限制 (比力, m/s²)
# 悬停时 T = g ≈ 9.81 m/s²
T_MAX = 20.0            # 最大推力比力 ≈ 2g (m/s²)
T_MIN = 2.0             # 最小推力比力 ≈ 0.2g (m/s²)
T_HOVER = G_VAL         # 悬停推力比力 = g ≈ 9.81 m/s²

# 角速度限制
RATE_MAX_XY = 3.0       # 最大Roll/Pitch角速度 (rad/s)
RATE_MAX_Z = 2.0        # 最大Yaw角速度 (rad/s)

# MPC时域配置
N_HORIZON = 20          # 预测步数
T_HORIZON = 1.0         # 预测时域 (秒)
# 每步时长: dt = T_HORIZON / N_HORIZON = 0.05s

# 代价函数权重 (可调参数)
# 位置权重 [x, y, z] - z权重更高保持高度
Q_POS = [100.0, 100.0, 200.0]

# 四元数权重 [qw, qx, qy, qz]
# 重要：设为0或很小，让MPC自由决定roll/pitch来追踪位置
# 不追踪四元数，因为参考四元数只有yaw，没有正确的roll/pitch倾斜
# yaw追踪通过单独的机制实现（后续改进）
Q_QUAT = [0.0, 0.0, 0.0, 0.0]

# 速度权重 [vx, vy, vz]
Q_VEL = [10.0, 10.0, 10.0]

# 输入权重 [Thrust, wx, wy, wz]
# 增加角速度惩罚，防止疯狂旋转
R_INPUT = [0.1, 1.0, 1.0, 0.5]



def export_drone_model():
    """
    定义无人机刚体动力学模型

    状态向量 x (10维):
        [px, py, pz, qw, qx, qy, qz, vx, vy, vz]
        - p: 位置 (ENU坐标系)
        - q: 四元数 (w, x, y, z顺序)
        - v: 速度

    控制向量 u (4维):
        [T, wx, wy, wz]
        - T: 推力 (Newtons)
        - w: 机体角速度 (rad/s)

    坐标系约定:
        - 使用ENU (East-North-Up) - 与ROS/MAVROS一致
        - Z轴向上为正
        - 推力沿机体+Z方向（向上）
    """
    model_name = 'ship_painter_mpc'

    # 状态变量定义
    # 位置 (ENU)
    px = SX.sym('px')
    py = SX.sym('py')
    pz = SX.sym('pz')

    # 四元数 (Hamilton convention: w, x, y, z)
    qw = SX.sym('qw')
    qx = SX.sym('qx')
    qy = SX.sym('qy')
    qz = SX.sym('qz')

    # 速度 (ENU)
    vx = SX.sym('vx')
    vy = SX.sym('vy')
    vz = SX.sym('vz')

    # 组合状态向量 [10x1]
    x = vertcat(px, py, pz, qw, qx, qy, qz, vx, vy, vz)

    # 控制变量定义
    T = SX.sym('T')      # 推力 (N)
    wx = SX.sym('wx')    # Roll角速度 (rad/s)
    wy = SX.sym('wy')    # Pitch角速度 (rad/s)
    wz = SX.sym('wz')    # Yaw角速度 (rad/s)

    u = vertcat(T, wx, wy, wz)

    # 动力学方程

    # 1. 位置导数 = 速度
    p_dot = vertcat(vx, vy, vz)

    # 2. 四元数 -> 旋转矩阵 R_WB (World <- Body)
    # 标准四元数到旋转矩阵的转换公式
    R_WB = vertcat(
        1 - 2*(qy**2 + qz**2),  2*(qx*qy - qw*qz),      2*(qx*qz + qw*qy),
        2*(qx*qy + qw*qz),      1 - 2*(qx**2 + qz**2),  2*(qy*qz - qw*qx),
        2*(qx*qz - qw*qy),      2*(qy*qz + qw*qx),      1 - 2*(qx**2 + qy**2)
    ).reshape((3, 3))

    # 3. 速度导数 (牛顿第二定律)
    # 使用rpg_mpc相同的动力学模型
    # T是比力(specific thrust = 加速度), 不是力!
    # a = R * [0,0,T] - [0,0,g]
    # 直接展开旋转矩阵第三列 (body Z轴在世界坐标系中的方向)
    v_dot = vertcat(
        2 * (qw * qy + qx * qz) * T,           # a_x
        2 * (qy * qz - qw * qx) * T,           # a_y
        (1 - 2*qx*qx - 2*qy*qy) * T - G_VAL    # a_z (减去重力)
    )

    # 4. 四元数导数 (四元数运动学)
    # q_dot = 0.5 * q ⊗ [0, omega]
    # 展开后:
    q_dot = 0.5 * vertcat(
        -qx*wx - qy*wy - qz*wz,      # dqw/dt
         qw*wx + qy*wz - qz*wy,      # dqx/dt
         qw*wy - qx*wz + qz*wx,      # dqy/dt
         qw*wz + qx*wy - qy*wx       # dqz/dt
    )

    # 组合状态导数
    x_dot = vertcat(p_dot, q_dot, v_dot)

    # 创建Acados模型 
    model = AcadosModel()
    model.f_expl_expr = x_dot       # 显式ODE: dx/dt = f(x, u)
    model.x = x                     # 状态
    model.u = u                     # 控制
    model.name = model_name

    return model


def setup_ocp():
    """
    配置最优控制问题 (OCP)

    优化目标:
        min ∑_{k=0}^{N-1} ||y_k - y_ref_k||_W^2 + ||y_N - y_ref_N||_{W_e}^2

    其中:
        y = [x; u] (14维)
        y_ref: 参考轨迹
        W, W_e: 权重矩阵

    约束:
        x_0 = x_current (初始状态)
        u_min <= u <= u_max (输入约束)
    """
    ocp = AcadosOcp()
    model = export_drone_model()
    ocp.model = model

    nx = model.x.size()[0]  # 10
    nu = model.u.size()[0]  # 4
    ny = nx + nu            # 14
    ny_e = nx               # 10 (终端)

    # 时域配置
    ocp.dims.N = N_HORIZON
    ocp.solver_options.tf = T_HORIZON

    # 代价函数配置
    # 使用 NONLINEAR_LS - 适合四元数非线性
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'  # 终端代价

    # 定义输出函数 y = [x; u]
    ocp.model.cost_y_expr = vertcat(model.x, model.u)
    ocp.model.cost_y_expr_e = model.x  # 终端只有状态

    # 构建权重矩阵
    Q_diag = np.array(Q_POS + Q_QUAT + Q_VEL)
    R_diag = np.array(R_INPUT)

    Q_mat = np.diag(Q_diag)
    R_mat = np.diag(R_diag)

    # W = diag(Q, R) for running cost
    ocp.cost.W = block_diag(Q_mat, R_mat)
    ocp.cost.W_e = Q_mat  # 终端只惩罚状态

    # 初始参考轨迹 (运行时会更新)
    y_ref = np.zeros(ny)
    y_ref[3] = 1.0          # qw = 1 (单位四元数，无旋转)
    y_ref[10] = T_HOVER     # 参考推力 = 悬停推力

    ocp.cost.yref = y_ref
    ocp.cost.yref_e = y_ref[:nx]

    # 输入约束
    ocp.constraints.lbu = np.array([T_MIN, -RATE_MAX_XY, -RATE_MAX_XY, -RATE_MAX_Z])
    ocp.constraints.ubu = np.array([T_MAX, RATE_MAX_XY, RATE_MAX_XY, RATE_MAX_Z])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])

    # 初始状态约束
    # x_0 = x_current (会在运行时设置)
    x0 = np.zeros(nx)
    x0[3] = 1.0  # qw = 1
    ocp.constraints.x0 = x0

    # 求解器选项
    # QP求解器: HPIPM (高性能内点法)
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'

    # Hessian近似: Gauss-Newton (适合最小二乘)
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'

    # 积分器: 显式Runge-Kutta
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_num_stages = 4    # RK4
    ocp.solver_options.sim_method_num_steps = 3     # 每步3个子步

    # NLP求解器: SQP-RTI (实时迭代)
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.nlp_solver_max_iter = 1      # RTI只需1次迭代

    # 打印级别 (0=无, 1=少量, 2=详细)
    ocp.solver_options.print_level = 0

    return ocp


def main():
    """
    主函数: 配置OCP并生成C代码
    """
    print("=" * 60)
    print("Ship Painter MPC Solver Generator")
    print("=" * 60)

    # 打印配置信息
    print(f"\n[配置参数]")
    print(f"  重力加速度: {G_VAL} m/s²")
    print(f"  比力范围: [{T_MIN}, {T_MAX}] m/s² (specific thrust)")
    print(f"  悬停比力: {T_HOVER:.2f} m/s² (= g)")
    print(f"  角速度限制: XY={RATE_MAX_XY} rad/s, Z={RATE_MAX_Z} rad/s")
    print(f"  预测时域: {T_HORIZON} s, {N_HORIZON}步")
    print(f"  每步时长: {T_HORIZON/N_HORIZON:.3f} s")

    print(f"\n[代价权重]")
    print(f"  位置 Q_pos: {Q_POS}")
    print(f"  姿态 Q_quat: {Q_QUAT}")
    print(f"  速度 Q_vel: {Q_VEL}")
    print(f"  输入 R_input: {R_INPUT}")

    # 配置OCP
    print(f"\n[生成中...] 配置最优控制问题")
    ocp = setup_ocp()

    # 生成代码
    print(f"[生成中...] 生成C代码")
    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'c_generated_code')

    # 创建求解器（自动生成代码）
    solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')

    print("\n" + "=" * 60)
    print("生成成功!")
    print("=" * 60)
    print(f"\n输出目录: {output_dir}")
    print(f"JSON配置: {os.path.join(os.path.dirname(os.path.abspath(__file__)), 'acados_ocp.json')}")

    print(f"\n[下一步]")
    print(f"  1. 复制生成的代码到ship_painter:")
    print(f"     cp -r c_generated_code ../src/mpc/")
    print(f"  2. 修改CMakeLists.txt添加Acados编译")
    print(f"  3. 编写mpc_runner_node.cpp")

    # 简单测试求解器
    print(f"\n[测试] 求解器初始化测试...")
    status = solver.solve()
    if status == 0:
        print("  求解器测试通过!")
        u0 = solver.get(0, "u")
        print(f"  测试输出 u0: T={u0[0]:.2f}N, wx={u0[1]:.3f}, wy={u0[2]:.3f}, wz={u0[3]:.3f}")
    else:
        print(f"  求解器测试失败，状态码: {status}")

    return solver


if __name__ == '__main__':
    main()
