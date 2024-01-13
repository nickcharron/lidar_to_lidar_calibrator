#!/bin/bash
set -e

if [ $# != 2 ]; then
  echo "Invalid number of inputs (provided: $#, expected: 2)"
  echo "Usage:"
  echo "bash /path_to/run_example.sh {path_to_catkin_ws} {path_to_daa}"
  exit
fi

cwd=$(pwd)
workspace_path=$1
data_path=$2
output_path="${data_path}/results"
example="parking_garage_vlp16s"

mkdir -p $output_path
cd $workspace_path

cmd="./build/lidar_to_lidar_calibrator/lidar_to_lidar_calibrator_main"
cmd="${cmd} -bag ${data_path}/lidar_data.bag"
cmd="${cmd} -config ${workspace_path}/src/lidar_to_lidar_calibrator/examples/${example}/config.json"
cmd="${cmd} -extrinsics ${workspace_path}/src/lidar_to_lidar_calibrator/examples/${example}/extrinsics.json"
cmd="${cmd} -matcher_config ${workspace_path}/src/lidar_to_lidar_calibrator/examples/${example}/icp.json"
cmd="${cmd} -poses ${data_path}/poses.json"
cmd="${cmd} -output ${output_path}/calibration_results.json"
cmd="${cmd} -maps_output_dir ${output_path}"
cmd="${cmd} -display=false"

echo ""
echo "Running command: "
echo ""
echo "$cmd"
echo ""
$cmd

cd $cwd