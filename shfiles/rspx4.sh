#!/bin/bash

# 1. 启动 RealSense 相机 (使用 run_d455.launch)
# 这里的 & 表示后台运行，sleep 5 是为了等相机初始化完再连飞控，防止USB冲突
roslaunch ship_painter run_d455.launch & 
sleep 5

# 2. 启动 MAVROS (连接飞控)
# 注意：/dev/ttyACM0 是飞控的串口号，如果不通请检查 ls /dev/tty*
# 921600 是 Pixhawk 4/6C 等飞控的常用波特率，如果是旧版可能是 57600
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
rosservice call /mavros/set_stream_rate 0 10 200 1

wait