#!/bin/bash
# ============================================================
# 数据集批量录制脚本
#
# 前提: boat.launch（Gazebo + PX4 SITL）已在另一终端运行
#
# 用法:
#   bash shfiles/batch_record.sh          # 默认每组 3 分钟
#   bash shfiles/batch_record.sh 5        # 每组 5 分钟
#   bash shfiles/batch_record.sh 1        # 短测（每组 1 分钟）
#
# 数据集输出: ship_painter/tmp/episode_NNN_*_.csv
# ============================================================

set -e

DURATION_MIN=${1:-3}
DURATION_SEC=$((DURATION_MIN * 60))
COOLDOWN=15  # 每组间隔等待秒数（让 PX4 重置 + 重新起飞）

PKG="ship_painter"
LAUNCH="mpc_test.launch"

# === 参数组合 ===

# 轨迹模式及对应 laps（确保轨迹持续时间 > episode 时长）
# hover: 3600s 无需调整
# circle: 2π*3/0.5 ≈ 37.7s/圈 → 6 圈 ≈ 226s
# yaw_spin: 4π*laps/0.5 → 8 圈 ≈ 201s
# figure8: 类似 circle → 6 圈
MODES=(0 1 2 3)
MODE_NAMES=("hover" "circle" "yaw_spin" "figure8")
MODE_LAPS_ARGS=("" "circle_laps:=6" "yaw_spin_laps:=8" "figure8_laps:=6")

# 风力配置（单轴上限 2N）
WIND_ARGS=(
    "wind_enabled:=false"
    "wind_enabled:=true wind_sigma_x:=0.5 wind_sigma_y:=0.5"
    "wind_enabled:=true wind_sigma_x:=1.0 wind_sigma_y:=1.0"
)
WIND_NAMES=("nowind" "wind_low" "wind_high")

# 机体力配置（不超过 2.2N）
BF_ARGS=(
    "body_force_enabled:=false"
    "body_force_enabled:=true body_force_x:=-0.8"
    "body_force_enabled:=true body_force_x:=-1.5"
    "body_force_enabled:=true body_force_x:=-2.2"
)
BF_NAMES=("nobf" "bf0.8" "bf1.5" "bf2.2")

TOTAL=$((${#MODES[@]} * ${#WIND_ARGS[@]} * ${#BF_ARGS[@]}))
COUNT=0

echo "============================================================"
echo "  批量数据集录制"
echo "  总组合数: $TOTAL"
echo "  每组时长: ${DURATION_MIN} 分钟"
echo "  预计总时长: $(( TOTAL * (DURATION_MIN * 60 + COOLDOWN) / 60 )) 分钟"
echo "============================================================"
echo ""

for mi in "${!MODES[@]}"; do
    for wi in "${!WIND_ARGS[@]}"; do
        for bi in "${!BF_ARGS[@]}"; do
            COUNT=$((COUNT + 1))
            echo "--- [$COUNT/$TOTAL] ${MODE_NAMES[$mi]} + ${WIND_NAMES[$wi]} + ${BF_NAMES[$bi]} ---"

            # 启动 roslaunch（后台，不经过管道以获得正确 PID）
            roslaunch $PKG $LAUNCH \
                test_mode:=${MODES[$mi]} \
                ${MODE_LAPS_ARGS[$mi]} \
                ${WIND_ARGS[$wi]} \
                ${BF_ARGS[$bi]} \
                &>"$(rospack find $PKG)/tmp/episode_${COUNT}.log" &

            LAUNCH_PID=$!
            echo "  roslaunch PID: $LAUNCH_PID"

            # 等待指定时长
            sleep $DURATION_SEC

            # 优雅关闭: SIGINT → ROS shutdown → 析构函数 → CSV flush
            echo "  Sending SIGINT to roslaunch ($LAUNCH_PID)..."
            kill -INT $LAUNCH_PID 2>/dev/null || true

            # 等待 roslaunch 优雅退出（最多 15 秒）
            for i in $(seq 1 15); do
                if ! kill -0 $LAUNCH_PID 2>/dev/null; then
                    break
                fi
                sleep 1
            done

            # 如果还没退出，强制 kill
            if kill -0 $LAUNCH_PID 2>/dev/null; then
                echo "  [WARN] roslaunch did not exit gracefully, force killing..."
                kill -9 $LAUNCH_PID 2>/dev/null || true
            fi

            # 清理可能残留的子节点
            pkill -f "mpc_runner_node.*__name:=mpc_runner" 2>/dev/null || true
            pkill -f "mpc_trajectory_tester_node" 2>/dev/null || true
            pkill -f "thrust_mapper_node" 2>/dev/null || true

            wait $LAUNCH_PID 2>/dev/null || true

            echo "  ✓ Episode $COUNT/$TOTAL complete"
            echo ""

            # 冷却期：让 PX4 重置状态
            sleep $COOLDOWN
        done
    done
done

echo "============================================================"
echo "  全部 $TOTAL 组录制完成"
echo "  数据集目录: $(rospack find ship_painter)/tmp/"
echo "============================================================"
ls -lt "$(rospack find ship_painter)/tmp/"*.csv 2>/dev/null | head -20
