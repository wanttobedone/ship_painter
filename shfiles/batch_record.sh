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
COOLDOWN=10  # 每组间隔等待秒数（让 PX4 重置）

PKG="ship_painter"
LAUNCH="mpc_test.launch"

# === 参数组合 ===

# 轨迹模式
MODES=(0 1 2 3)
MODE_NAMES=("hover" "circle" "yaw_spin" "figure8")

# 风力配置
WIND_ARGS=(
    "wind_enabled:=false"
    "wind_enabled:=true wind_sigma_x:=0.5 wind_sigma_y:=0.5"
    "wind_enabled:=true wind_sigma_x:=2.0 wind_sigma_y:=2.0"
)
WIND_NAMES=("nowind" "wind_low" "wind_high")

# 机体力配置
BF_ARGS=(
    "body_force_enabled:=false"
    "body_force_enabled:=true body_force_x:=-2.0"
    "body_force_enabled:=true body_force_x:=-5.0"
    "body_force_enabled:=true body_force_x:=-8.0"
)
BF_NAMES=("nobf" "bf2" "bf5" "bf8")

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

            # 启动 roslaunch（后台）
            roslaunch $PKG $LAUNCH \
                test_mode:=${MODES[$mi]} \
                ${WIND_ARGS[$wi]} \
                ${BF_ARGS[$bi]} \
                2>&1 | sed 's/^/  /' &

            LAUNCH_PID=$!

            # 等待指定时长
            sleep $DURATION_SEC

            # 优雅关闭（SIGINT → ROS shutdown → DatasetCollector flush）
            kill -INT $LAUNCH_PID 2>/dev/null || true
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
