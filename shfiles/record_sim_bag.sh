#!/bin/bash
# Record an RGB-D rosbag from the Gazebo simulation that mirrors the real
# D455F field-capture bag, so the offline RTAB-Map + Python pipeline can
# be reused with no code changes.
#
# Usage:
#   ./record_sim_bag.sh                       # writes sim_building_scan.bag
#   ./record_sim_bag.sh my_scan.bag           # custom name
#   ./record_sim_bag.sh my_scan.bag /tmp      # custom name + dir
#
# Topics included match the realsense2_camera ROS driver layout used in the
# real-flight workflow, plus /clock (use_sim_time=true during playback).

set -e

BAG_NAME="${1:-sim_building_scan.bag}"
OUT_DIR="${2:-$(pwd)}"

mkdir -p "$OUT_DIR"
cd "$OUT_DIR"

echo "[record_sim_bag] writing $OUT_DIR/$BAG_NAME"
echo "[record_sim_bag] Ctrl+C to stop."

exec rosbag record -O "$BAG_NAME" \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /camera/aligned_depth_to_color/image_raw \
  /camera/aligned_depth_to_color/camera_info \
  /tf \
  /tf_static \
  /clock
