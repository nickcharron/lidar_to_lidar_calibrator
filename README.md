# lidar_to_lidar_calibrator

**tl;dr**: A simple tool for calibration two lidars to each other using a single high-rate trajectory (e.g., from SLAM). This works by using the trajectory to aggregate local scans for each lidar across the entire trajectory, then running scan registration between all aggregated scans and optimizing for the calibration that minimizes the sum of these scan registration measurements.

## Goal

TODO 

**Caviats**: 

* Must have high-rate (e.g., IMU rate) trajectory

TODO 

## Dependencies

This repo only depends on our internal [libbeam](https://github.com/BEAMRobotics/libbeam) library. 

Libeam depends on the following:

* [ROS](https://www.ros.org/)
* [Catch2](https://github.com/catchorg/Catch2)
* [Eigen3](https://gitlab.com/libeigen/eigen/)
* [PCL 1.11.1 or greater](https://github.com/PointCloudLibrary/pcl)
* [gflags](https://github.com/gflags/gflags)
* [nlohmann json](https://github.com/nlohmann/json)

For more information on libbeam and it's dependencies, see the [docs](https://github.com/BEAMRobotics/libbeam). 

## Install

First, install the libbeam dependencies. We have a [script](https://github.com/BEAMRobotics/libbeam/blob/master/scripts/install.bash) to help make this easier which you can run, or just copy commands from.

We recommend using catkin to build this tool and libbeam.

```
mkdir -p ~/lidar2lidarcalib_catkin_ws/src
cd ~/lidar2lidarcalib_catkin_ws
catkin build -j2
```

Clone libbeam and this repo:

```
cd ~/cam2mapcalib_catkin_ws/src
git clone --depth 1 https://github.com/BEAMRobotics/libbeam.git
git clone --depth 1 https://github.com/nickcharron/lidar_to_lidar_calibrator.git
```

Build:

```
cd ~/lidar2lidarcalib_catkin_ws/
catkin build -j2
```

## Running the tool

For information on required inputs to the binary and their formats, run the binary with -help flag:

```
cd ~/lidar2lidarcalib_catkin_ws/
./build/lidar_to_lidar_calibrator/lidar_to_lidar_calibrator_main -help
```

TODO 

## Example Tutorial

TODO 