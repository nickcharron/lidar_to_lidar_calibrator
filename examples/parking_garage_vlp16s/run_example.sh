#!/bin/bash
set -e

cwd=$(pwd)

cd ~/catkin_ws

./build/lidar_to_lidar_calibrator/lidar_to_lidar_calibrator_main \
-bag ~/d/inspection.bag \
-config ~/catkin_ws/src/beam_robotics/inspection_tools/lidar_to_lidar_calibrator/config/config.json \
-extrinsics ~/catkin_ws/src/beam_robotics/calibration/results/inspector_gadget2/current/extrinsics/extrinsics.json \
-matcher_config ~/catkin_ws/src/beam_robotics/inspection_tools/lidar_to_lidar_calibrator/config/icp.json \
-poses ~/d/results/map_builder/final_poses.json \
-output ~/d/results/lidar_to_lidar_calib/calibration_results.json \
-maps_output_dir ~/d/results/lidar_to_lidar_calib/ \
-display=false

cd $cwd