#include <gflags/gflags.h>

#include <beam_utils/gflags.h>
#include <lidar_to_lidar_calibrator/LidarToLidarCalibrator.h>

DEFINE_string(
    poses, "",
    "Full path poses file (Required). See pose file types in "
    "libbeam/beam_mapping/poses.h. Or extract poses using 3d_map_builder");
DEFINE_validator(poses, &beam::gflags::ValidateJsonFileMustExist);

DEFINE_string(extrinsics, "",
              "Full path to extrinsics json (Required). See "
              "beam_robotics/calibration for file formats");
DEFINE_validator(extrinsics, &beam::gflags::ValidateJsonFileMustExist);

DEFINE_string(config, "",
              "Full path to config file. For example config, see "
              "lidar_to_lidar_calibrator/config/config.json");
DEFINE_validator(config, &beam::gflags::ValidateJsonFileMustExist);

DEFINE_string(matcher_config, "",
              "Full path to matcher config file. For example config, see "
              "lidar_to_lidar_calibrator/config/icp.json, or see "
              "libbeam/beam_matching/config");
DEFINE_validator(matcher_config, &beam::gflags::ValidateJsonFileMustExist);

DEFINE_string(bag, "", "Full path to bag file which contains the lidar data");
DEFINE_validator(bag, &beam::gflags::ValidateBagFileMustExist);

DEFINE_string(output, "",
              "Full path to output file (Required). Example: "
              "/home/user/new_extrinsics.json");
DEFINE_validator(output, &beam::gflags::ValidateMustBeJson);

DEFINE_string(maps_output_dir, "",
              "[Optional] Full path to output directory for maps. If empty, "
              "maps will not be output. Otherwise, we will output each "
              "measurement in the world frame");

DEFINE_bool(
    display, false,
    "Set to true to visualize results and accept/reject each measurement");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  LidarToLidarCalibrator::Inputs inputs{
      .poses = FLAGS_poses,
      .extrinsics = FLAGS_extrinsics,
      .config = FLAGS_config,
      .matcher_config = FLAGS_matcher_config,
      .bag = FLAGS_bag,
      .output = FLAGS_output,
      .maps_output_dir = FLAGS_maps_output_dir,
      .display = FLAGS_display,
  };
  LidarToLidarCalibrator calibrator(inputs);
  calibrator.Run();
  return 0;
}