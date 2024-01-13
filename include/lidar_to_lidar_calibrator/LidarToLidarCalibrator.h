#pragma once

#include <pcl/visualization/pcl_visualizer.h>
#include <rosbag/bag.h>

#include <beam_calibration/TfTree.h>
#include <beam_mapping/Poses.h>
#include <beam_matching/Matchers.h>
#include <beam_utils/pointclouds.h>

using PCLViewer = pcl::visualization::PCLVisualizer::Ptr;
using ColorHandler =
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>;

class LidarToLidarCalibrator
{
public:
  struct Inputs
  {
    std::string poses;
    std::string extrinsics;
    std::string config;
    std::string matcher_config;
    std::string bag;
    std::string output;
    std::string maps_output_dir;
    bool display;
  };

  struct LidarConfig
  {
    std::string frame_id;
    std::string topic;
    Eigen::Matrix4d T_B_L; // T_Baselink_Lidar initial
  };

  struct PoseThreshold
  {
    double rotation_deg;
    double translation_m;
  };

  struct Config
  {
    Eigen::Vector3f voxel_size;
    double rotation_thresh_rad;
    double trans_thresh_m;
    LidarConfig L1;
    LidarConfig L2;
    PoseThreshold registration; // or criteria
    PoseThreshold aggregation;  // or criteria
    std::string lidar_type;     // VELODYNE OR OUSTER

    void LoadFromJson(const std::string &config_path);
  };

  struct Measurement
  {
    Eigen::Matrix4d T_Lidar1_Lidar2;
    double error_rot_rad;
    double error_trans_m;
    ros::Time start;
    ros::Time end;
    bool valid{false};
  };

  explicit LidarToLidarCalibrator(const Inputs &inputs);

  ~LidarToLidarCalibrator() = default;

  void Run();

private:
  void LoadPoses();

  void LoadExtrinsics();

  void GetMeasurementTimes();

  void GetMeasurements();

  void GetMeasurement(Measurement &m);

  PointCloudPtr AggregateScansFromBag(const ros::Time &start_time,
                                      const ros::Time &end_time,
                                      const std::string &topic,
                                      const Eigen::Matrix4d &T_B_L) const;

  void UpdateViewer();

  void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event);

  void DisplayInstructions();

  void RejectBadMeasurements();

  void Solve();

  void OutputResults();

  void OutputMaps() const;

  Inputs inputs_;
  Config config_;

  pcl::visualization::PCLVisualizer::Ptr viewer_;
  PointCloud::Ptr lidar1_;
  PointCloud::Ptr lidar2_initial_;
  PointCloud::Ptr lidar2_aligned_;
  beam_calibration::TfTree poses_tree_;
  beam_mapping::Poses poses_;
  beam_calibration::TfTree extrinsics_;
  bool save_measurement_{true};
  bool next_measurement_{false};
  bool solve_{false};
  std::vector<Measurement> measurements_;
  std::unique_ptr<beam_matching::Matcher<PointCloudPtr>> matcher_;

  Eigen::Matrix4d T_L1_L2_Init_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d T_L1_L2_Opt_{Eigen::Matrix4d::Identity()};

  rosbag::Bag bag_;

  // params
  std::vector<double> backgound_rgb_{0.8, 0.8, 0.8};
  double min_pose_rate_hz_{30};
};
