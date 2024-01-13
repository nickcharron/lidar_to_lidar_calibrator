#include <lidar_to_lidar_calibrator/LidarToLidarCalibrator.h>

#include <filesystem>

#include <nlohmann/json.hpp>
#include <pcl/common/transforms.h>
#include <rosbag/view.h>

#include <beam_filtering/VoxelDownsample.h>
#include <beam_optimization/CeresParams.h>
#include <beam_utils/pointclouds.h>
#include <beam_utils/se3.h>

#include <lidar_to_lidar_calibrator/ceres_pose_prior_cost_functor.h>

namespace {
nlohmann::json GetTransformJson(const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Vector3d t = T.block(0, 3, 3, 1);
  Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2);
  Eigen::Quaterniond q(R);

  nlohmann::json J;
  J["T"] = beam::EigenTransformToVector(T);
  J["txyz_m"] = std::vector<double>{t[0], t[1], t[2]};
  J["qwxyz"] = std::vector<double>{q.w(), q.x(), q.y(), q.z()};
  J["RPY_deg"] = std::vector<double>{
      beam::Rad2Deg(rpy[0]), beam::Rad2Deg(rpy[1]), beam::Rad2Deg(rpy[2])};
  return J;
}
} // namespace

using namespace beam_matching;

void LidarToLidarCalibrator::Config::LoadFromJson(
    const std::string& config_path) {
  BEAM_INFO("Reading config from: {}", config_path);
  nlohmann::json J;
  if (!beam::ReadJson(config_path, J)) {
    throw std::runtime_error{"invalid json file path"};
  }
  beam::ValidateJsonKeysOrThrow(
      {"lidar_1", "lidar_2", "registration_error_thresholds",
       "aggregation_movement_thresholds", "voxel_downsample", "lidar_type"},
      J);
  nlohmann::json J_L1 = J["lidar_1"];
  beam::ValidateJsonKeysOrThrow({"frame_id", "topic"}, J_L1);
  L1.frame_id = J_L1["frame_id"];
  L1.topic = J_L1["topic"];

  nlohmann::json J_L2 = J["lidar_2"];
  beam::ValidateJsonKeysOrThrow({"frame_id", "topic"}, J_L2);
  L2.frame_id = J_L2["frame_id"];
  L2.topic = J_L2["topic"];

  nlohmann::json J_RegThres = J["registration_error_thresholds"];
  beam::ValidateJsonKeysOrThrow({"translation_m", "rotation_deg"}, J_RegThres);
  registration.translation_m = J_RegThres["translation_m"];
  registration.rotation_deg = J_RegThres["rotation_deg"];

  nlohmann::json J_AggThres = J["aggregation_movement_thresholds"];
  beam::ValidateJsonKeysOrThrow({"translation_m", "rotation_deg"}, J_AggThres);
  aggregation.translation_m = J_AggThres["translation_m"];
  aggregation.rotation_deg = J_AggThres["rotation_deg"];

  std::vector<double> vox = J["voxel_downsample"];
  if (vox.size() != 3) {
    BEAM_CRITICAL(
        "Invalid voxel_downsample config, must be a vector of size 3!");
    throw std::invalid_argument{"Invalid input"};
  }
  voxel_size = Eigen::Vector3f(vox[0], vox[1], vox[2]);

  lidar_type = J["lidar_type"];
}

LidarToLidarCalibrator::LidarToLidarCalibrator(const Inputs& inputs)
    : inputs_(inputs) {
  config_.LoadFromJson(inputs_.config);
  LoadPoses();
  LoadExtrinsics();
  BEAM_INFO("Done initializing LidarToLidarCalibrator");
}

void LidarToLidarCalibrator::LoadPoses() {
  // Load previous poses file specified in labeler json
  BEAM_INFO("Loading poses form {}", inputs_.poses);
  poses_.LoadFromFile(inputs_.poses);
  const auto& poses = poses_.GetPoses();
  const auto& timestamps = poses_.GetTimeStamps();
  BEAM_INFO("Filling TF tree with {} poses", poses.size());
  for (int i = 0; i < poses.size(); i++) {
    Eigen::Affine3d T(poses[i]);
    poses_tree_.AddTransform(T, poses_.GetFixedFrame(), poses_.GetMovingFrame(),
                             timestamps[i]);
  }
}

void LidarToLidarCalibrator::LoadExtrinsics() {
  BEAM_INFO("Loading extrinsic from {}", inputs_.extrinsics);
  extrinsics_.LoadJSON(inputs_.extrinsics);

  config_.L1.T_B_L =
      extrinsics_
          .GetTransformEigen(poses_.GetMovingFrame(), config_.L1.frame_id)
          .matrix();
  config_.L2.T_B_L =
      extrinsics_
          .GetTransformEigen(poses_.GetMovingFrame(), config_.L2.frame_id)
          .matrix();
  T_L1_L2_Init_ =
      extrinsics_.GetTransformEigen(config_.L1.frame_id, config_.L2.frame_id)
          .matrix();
}

void LidarToLidarCalibrator::Run() {
  // Load bag
  BEAM_INFO("Loading bag: {}", inputs_.bag);
  try {
    bag_.open(inputs_.bag, rosbag::bagmode::Read);
  } catch (rosbag::BagException& ex) {
    BEAM_CRITICAL("Bag exception : {}}", ex.what());
    throw std::runtime_error{"invalid bag"};
  }

  if (inputs_.display) {
    viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>();
    std::function<void(const pcl::visualization::KeyboardEvent&)> keyboard_cb =
        [this](const pcl::visualization::KeyboardEvent& event) {
          KeyboardEventOccurred(event);
        };
    viewer_->registerKeyboardCallback(keyboard_cb);
  }

  // setup matcher
  const auto& m_conf = inputs_.matcher_config;
  auto matcher_type = GetTypeFromConfig(m_conf);
  if (matcher_type == MatcherType::LOAM) {
    BEAM_CRITICAL("LOAM matcher type not implemented");
    throw std::runtime_error{"invalid matcher config"};
  } else if (matcher_type == MatcherType::ICP) {
    matcher_ = std::make_unique<IcpMatcher>(IcpMatcher::Params(m_conf));
  } else if (matcher_type == MatcherType::GICP) {
    matcher_ = std::make_unique<GicpMatcher>(GicpMatcher::Params(m_conf));
  } else if (matcher_type == MatcherType::NDT) {
    matcher_ = std::make_unique<NdtMatcher>(NdtMatcher::Params(m_conf));
  } else {
    BEAM_ERROR("Invalid matcher type");
    throw std::invalid_argument{"invalid json"};
  }

  GetMeasurementTimes();
  GetMeasurements();
  RejectBadMeasurements();
  Solve();
  OutputResults();
}

void LidarToLidarCalibrator::GetMeasurementTimes() {
  BEAM_INFO(
      "Getting all measurement time ranges given the input trajectory and "
      "aggregation movement thresholds");

  const auto& poses = poses_.GetPoses();
  const auto& timestamps = poses_.GetTimeStamps();

  measurements_.emplace_back(Measurement());
  measurements_.back().start = timestamps.front();
  Eigen::Matrix4d T_W_B_last = poses.front();
  for (int i = 1; i < timestamps.size(); i++) {
    if (beam::PassedMinMotion(T_W_B_last, poses.at(i),
                              config_.aggregation.rotation_deg,
                              config_.aggregation.translation_m)) {
      T_W_B_last = poses.at(i);
      measurements_.back().end = timestamps.at(i);
      measurements_.emplace_back(Measurement());
      measurements_.back().start = timestamps.at(i);
    }
  }
  measurements_.pop_back();
  BEAM_INFO("Loaded {} measurement windows with total time range: [{}, {}]s",
            measurements_.size(),
            std::to_string(measurements_.front().start.toSec()),
            std::to_string(measurements_.back().end.toSec()));
}

void LidarToLidarCalibrator::GetMeasurements() {
  if (inputs_.display) { DisplayInstructions(); }

  for (int i = 0; i < measurements_.size(); i++) {
    BEAM_INFO("processing measurement {}/{}", i, measurements_.size());
    auto& m = measurements_.at(i);
    GetMeasurement(m);
    UpdateViewer();
    while (inputs_.display && !viewer_->wasStopped()) {
      viewer_->spinOnce(10);
      if (next_measurement_) {
        m.valid = save_measurement_;
        if (save_measurement_) {
          BEAM_INFO("Saving measurement {}", i);
        } else {
          BEAM_INFO("Rejecting measurement {}", i);
        }
        save_measurement_ = true;
        next_measurement_ = false;
        break;
      }
      if (solve_) {
        m.valid = save_measurement_;
        if (save_measurement_) {
          BEAM_INFO("Saving measurement {}", i);
        } else {
          BEAM_INFO("Rejecting measurement {}", i);
        }
        break;
      }
    }
    if (solve_) { break; }
  }
  BEAM_INFO("Done getting measurements");
}

void LidarToLidarCalibrator::KeyboardEventOccurred(
    const pcl::visualization::KeyboardEvent& event) {
  if (event.getKeySym() == "y" && event.keyDown()) {
    next_measurement_ = true;
    save_measurement_ = true;
  } else if (event.getKeySym() == "n" && event.keyDown()) {
    next_measurement_ = true;
    save_measurement_ = false;
  } else if (event.getKeySym() == "s" && event.keyDown()) {
    solve_ = true;
  }
}

void LidarToLidarCalibrator::GetMeasurement(Measurement& m) {
  // get Lidar 1 aggregation in Lidar 1 frame
  lidar1_ =
      AggregateScansFromBag(m.start, m.end, config_.L1.topic, config_.L1.T_B_L);
  if (!lidar1_) { m.valid = false; }

  // get Lidar 2 aggregation in Lidar 1 frame
  PointCloudPtr lidar2_in_lidar2frame =
      AggregateScansFromBag(m.start, m.end, config_.L2.topic, config_.L2.T_B_L);
  if (!lidar2_in_lidar2frame) { m.valid = false; }

  // transform Lidar 2 scan into estimated lidar 1 frame
  lidar2_initial_ = std::make_shared<PointCloud>();
  pcl::transformPointCloud(*lidar2_in_lidar2frame, *lidar2_initial_,
                           Eigen::Affine3d(T_L1_L2_Init_));

  // run registration
  matcher_->SetRef(lidar1_);
  matcher_->SetTarget(lidar2_initial_);
  bool match_success = matcher_->Match();
  m.T_Lidar1_Lidar2 = matcher_->ApplyResult(T_L1_L2_Init_);
  lidar2_aligned_ = std::make_shared<PointCloud>();
  pcl::transformPointCloud(*lidar2_in_lidar2frame, *lidar2_aligned_,
                           Eigen::Affine3d(m.T_Lidar1_Lidar2));

  // check against max error thresholds
  Eigen::Matrix4d T_diff =
      T_L1_L2_Init_ * beam::InvertTransform(m.T_Lidar1_Lidar2);
  Eigen::Matrix3d R_diff = T_diff.block(0, 0, 3, 3);
  m.error_rot_rad = std::abs(Eigen::AngleAxis<double>(R_diff).angle());
  m.error_trans_m = T_diff.block(0, 3, 3, 1).norm();

  if (beam::Rad2Deg(m.error_rot_rad) > config_.registration.rotation_deg) {
    BEAM_ERROR(
        "Rotation ({}deg) result is higher than absolute threshold ({}deg)",
        beam::Rad2Deg(m.error_rot_rad), config_.registration.rotation_deg);
    m.valid = false;
  } else if (m.error_trans_m > config_.registration.translation_m) {
    BEAM_ERROR(
        "Translation ({}m) result is higher than absolute threshold ({}m)",
        m.error_trans_m, config_.registration.translation_m);
    m.valid = false;
  } else {
    m.valid = true;
  }
}

PointCloudPtr LidarToLidarCalibrator::AggregateScansFromBag(
    const ros::Time& start_time, const ros::Time& end_time,
    const std::string& topic, const Eigen::Matrix4d& T_B_L) const {
  PointCloudPtr aggregate = std::make_shared<PointCloud>();
  rosbag::View view(bag_, rosbag::TopicQuery(topic), start_time, end_time);
  if (view.size() == 0) {
    BEAM_WARN("Empty bag view with timestamp range: [{}, {}]s and topic: {}",
              std::to_string(start_time.toSec()),
              std::to_string(end_time.toSec()), topic);
    return nullptr;
  }

  // get start pose
  Eigen::Affine3d T_World_BaselinkStart = poses_tree_.GetTransformEigen(
      poses_.GetFixedFrame(), poses_.GetMovingFrame(), start_time);
  Eigen::Matrix4d T_LidarStart_World =
      beam::InvertTransform(T_B_L) * T_World_BaselinkStart.inverse().matrix();

  for (auto iter = view.begin(); iter != view.end(); iter++) {
    auto sensor_msg = iter->instantiate<sensor_msgs::PointCloud2>();
    if (!sensor_msg) {
      BEAM_CRITICAL("Invalid message type, check your topics");
      throw std::runtime_error{"invalid message type"};
    }
    ros::Time scan_time = sensor_msg->header.stamp;

    if (config_.lidar_type == "VELODYNE") {
      pcl::PointCloud<PointXYZIRT> cloud;
      beam::ROSToPCL(cloud, *sensor_msg);
      for (const auto& p : cloud.points) {
        // get pose at point time
        ros::Time pt = scan_time + ros::Duration(p.time);
        if (pt < start_time || pt > end_time) { continue; }
        Eigen::Affine3d T_World_BaselinkStart = poses_tree_.GetTransformEigen(
            poses_.GetFixedFrame(), poses_.GetMovingFrame(), pt);
        Eigen::Matrix4d T_LidarStart_LidarK =
            T_LidarStart_World * T_World_BaselinkStart * T_B_L;

        // convert to start time of aggregate and add
        Eigen::Vector4d p_deskewed =
            T_LidarStart_LidarK * Eigen::Vector4d(p.x, p.y, p.z, 1);
        pcl::PointXYZ p_new;
        p_new.x = p_deskewed[0];
        p_new.y = p_deskewed[1];
        p_new.z = p_deskewed[2];
        aggregate->points.push_back(p_new);
      }
    } else if (config_.lidar_type == "OUSTER") {
      pcl::PointCloud<PointXYZITRRNR> cloud;
      beam::ROSToPCL(cloud, *sensor_msg);
      for (const auto& p : cloud.points) {
        // get pose at point time
        ros::Time pt = scan_time + ros::Duration(p.time);
        Eigen::Affine3d T_World_BaselinkStart = poses_tree_.GetTransformEigen(
            poses_.GetFixedFrame(), poses_.GetMovingFrame(), pt);
        Eigen::Matrix4d T_LidarStart_LidarK =
            T_LidarStart_World * T_World_BaselinkStart * T_B_L;

        // convert to start time of aggregate and add
        Eigen::Vector4d p_deskewed =
            T_LidarStart_LidarK * Eigen::Vector4d(p.x, p.y, p.z, 1);
        pcl::PointXYZ p_new;
        p_new.x = p_deskewed[0];
        p_new.y = p_deskewed[1];
        p_new.z = p_deskewed[2];
        aggregate->points.push_back(p_new);
      }
    } else {
      BEAM_ERROR("Invalid input lidar type: {}, options: VELODYNE, OUSTER",
                 config_.lidar_type);
      throw std::runtime_error{"invalid input lidar type"};
    }
  }

  PointCloudPtr aggregate_filtered = std::make_shared<PointCloud>();
  beam_filtering::VoxelDownsample voxel_filter(config_.voxel_size);
  voxel_filter.SetInputCloud(aggregate);
  voxel_filter.Filter();
  *aggregate_filtered = voxel_filter.GetFilteredCloud();
  return aggregate_filtered;
}

void LidarToLidarCalibrator::UpdateViewer() {
  if (!inputs_.display) { return; }

  viewer_->removePointCloud("L1");
  viewer_->removePointCloud("L2_Est");
  viewer_->removePointCloud("L2_Opt");

  if (lidar1_ && lidar1_->size() > 0) {
    PointCloudColPtr l1_col = std::make_shared<PointCloudCol>();
    *l1_col = beam::ColorPointCloud(*lidar1_, 0, 0, 255);
    ColorHandler rgb_l1(l1_col);
    viewer_->addPointCloud<pcl::PointXYZRGB>(l1_col, rgb_l1, "L1");
  }

  if (lidar2_initial_ && lidar2_initial_->size() > 0) {
    PointCloudColPtr l2_est_col = std::make_shared<PointCloudCol>();
    *l2_est_col = beam::ColorPointCloud(*lidar2_initial_, 255, 0, 0);
    ColorHandler rgb_l2_est(l2_est_col);
    viewer_->addPointCloud<pcl::PointXYZRGB>(l2_est_col, rgb_l2_est, "L2_Est");
  }

  if (lidar2_aligned_ && lidar2_aligned_->size() > 0) {
    PointCloudColPtr l2_opt_col = std::make_shared<PointCloudCol>();
    *l2_opt_col = beam::ColorPointCloud(*lidar2_aligned_, 0, 255, 0);
    ColorHandler rgb_l2_opt(l2_opt_col);
    viewer_->addPointCloud<pcl::PointXYZRGB>(l2_opt_col, rgb_l2_opt, "L2_Opt");
  }
}

void LidarToLidarCalibrator::DisplayInstructions() {
  std::cout
      << "\nPress 'y' to accept measurement\n"
      << "Press 'n' to reject measurement\n"
      << "Press 's' to stop taking measurements and solve current "
         "measurements\n"
      << "Close viewer window to stop visualizing and accept default result\n"
      << "Blue: L1 aggregate in L1 frame\n"
      << "Red: L2 aggregate in L1 frame using initial extrinsics\n"
      << "Green: L2 aggregate in L1 after registration alignment\n";
}

void LidarToLidarCalibrator::RejectBadMeasurements() {
  // get mean errors
  int counter = 0;
  double rot_error_sum = 0;
  double trans_error_sum = 0;
  for (const auto& m : measurements_) {
    if (!m.valid) { continue; }
    counter++;
    rot_error_sum += m.error_rot_rad;
    trans_error_sum += m.error_trans_m;
  }
  double mean_error_rot = rot_error_sum / counter;
  double mean_error_trans = trans_error_sum / counter;

  // get standard deviations
  double rot_dev_sum_squared = 0;
  double trans_dev_sum_squared = 0;
  for (const auto& m : measurements_) {
    if (!m.valid) { continue; }
    rot_dev_sum_squared +=
        (m.error_rot_rad - mean_error_rot) * (m.error_rot_rad - mean_error_rot);
    trans_dev_sum_squared += (m.error_trans_m - mean_error_trans) *
                             (m.error_trans_m - mean_error_trans);
  }
  double std_dev_rot = std::sqrt(rot_dev_sum_squared / counter);
  double std_dev_trans = std::sqrt(trans_dev_sum_squared / counter);

  // reject measurements that are greater than mean + 2 x std. dev.
  double max_rot_error = mean_error_rot + 2 * std_dev_rot;
  double min_rot_error = mean_error_rot - 2 * std_dev_rot;
  double max_trans_error = mean_error_trans + 2 * std_dev_trans;
  double min_trans_error = mean_error_trans - 2 * std_dev_trans;
  for (int i = 0; i < measurements_.size(); i++) {
    auto& m = measurements_.at(i);
    if (!m.valid) { continue; }
    if (m.error_rot_rad > max_rot_error) {
      BEAM_INFO(
          "Removing measurement {} , rotation error ({} Deg) is too large", i,
          beam::Rad2Deg(m.error_rot_rad));
      m.valid = false;
    } else if (m.error_rot_rad < min_rot_error) {
      BEAM_INFO(
          "Removing measurement {} , rotation error ({} Deg) is too large", i,
          beam::Rad2Deg(m.error_rot_rad));
      m.valid = false;
    } else if (m.error_trans_m > max_trans_error) {
      BEAM_INFO(
          "Removing measurement {} , translation error ({} Deg) is too large",
          i, m.error_trans_m);
      m.valid = false;
    } else if (m.error_trans_m < min_trans_error) {
      BEAM_INFO(
          "Removing measurement {} , translation error ({} Deg) is too large",
          i, m.error_trans_m);
      m.valid = false;
    }
  }
}

void LidarToLidarCalibrator::Solve() {
  BEAM_INFO("Solving for calibration");

  // setup problem & params
  beam_optimization::CeresParams ceres_params;
  ceres_params.GetSolverOptionsMutable().minimizer_progress_to_stdout = true;
  std::shared_ptr<ceres::Problem> problem =
      std::make_shared<ceres::Problem>(ceres_params.ProblemOptions());
  std::unique_ptr<ceres::LossFunction> loss_function =
      ceres_params.LossFunction();

  // add calibration
  Eigen::Matrix3d R = T_L1_L2_Init_.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  Eigen::Vector3d t = T_L1_L2_Init_.block(0, 3, 3, 1);
  std::vector<double> p{t[0], t[1], t[2]};
  std::vector<double> o{q.w(), q.x(), q.y(), q.z()};

  std::unique_ptr<ceres::LocalParameterization> quat_parameterization(
      new ceres::QuaternionParameterization());
  std::unique_ptr<ceres::LocalParameterization> identity_parameterization(
      new ceres::IdentityParameterization(3));

  problem->AddParameterBlock(&(p[0]), 3, identity_parameterization.get());
  problem->AddParameterBlock(&(o[0]), 4, quat_parameterization.get());

  for (const Measurement& m : measurements_) {
    if (!m.valid) { continue; }
    std::unique_ptr<ceres::CostFunction> cost(
        CeresPosePriorCostFunctor::Create(m.T_Lidar1_Lidar2));
    problem->AddResidualBlock(cost.release(), loss_function.get(), &(p[0]),
                              &(o[0]));
  }

  // solve
  ceres::Solver::Summary ceres_summary;
  ceres::Solve(ceres_params.SolverOptions(), problem.get(), &ceres_summary);

  std::string report = ceres_summary.FullReport();
  std::cout << "\n\nCeres Report:\n\n" << report << "\n";
  if (!ceres_summary.IsSolutionUsable()) {
    BEAM_ERROR("Invalid calibration refinement solution");
  } else {
    BEAM_INFO("OPTIMIZATION SUCCESSFUL!");
  }

  Eigen::Quaterniond q_opt;
  q_opt.w() = o[0];
  q_opt.x() = o[1];
  q_opt.y() = o[2];
  q_opt.z() = o[3];
  Eigen::Matrix3d R_opt(q_opt);
  T_L1_L2_Opt_.block(0, 0, 3, 3) = R_opt;
  T_L1_L2_Opt_(0, 3) = p[0];
  T_L1_L2_Opt_(1, 3) = p[1];
  T_L1_L2_Opt_(2, 3) = p[2];
}

void LidarToLidarCalibrator::OutputResults() {
  BEAM_INFO("Outputting results");
  Eigen::Matrix4d T_diff = T_L1_L2_Init_ * beam::InvertTransform(T_L1_L2_Opt_);

  nlohmann::json J;
  J["to_frame"] = config_.L1.frame_id;
  J["from_frame"] = config_.L2.frame_id;
  nlohmann::json J_T_L1_L2;
  J_T_L1_L2["original"] = GetTransformJson(T_L1_L2_Init_);
  J_T_L1_L2["optimized"] = GetTransformJson(T_L1_L2_Opt_);
  J_T_L1_L2["difference"] = GetTransformJson(T_diff);
  J["T_L1_L2"] = J_T_L1_L2;

  BEAM_INFO("Saving results to: {}", inputs_.output);
  std::ofstream filejson(inputs_.output);
  filejson << std::setw(4) << J << std::endl;

  if (!inputs_.maps_output_dir.empty()) { OutputMaps(); }

  BEAM_INFO("Done outputting results");
}

void LidarToLidarCalibrator::OutputMaps() const {
  if (!std::filesystem::is_directory(inputs_.maps_output_dir)) {
    BEAM_ERROR("Invalid maps output directory: {}", inputs_.maps_output_dir);
    return;
  }

  for (int i = 0; i < measurements_.size(); i++) {
    BEAM_INFO("creating maps for measurement {}", i);
    auto& m = measurements_.at(i);

    PointCloudPtr l1 = AggregateScansFromBag(m.start, m.end, config_.L1.topic,
                                             config_.L1.T_B_L);

    PointCloudPtr l2 = AggregateScansFromBag(m.start, m.end, config_.L2.topic,
                                             config_.L2.T_B_L);

    // transform into world frame
    Eigen::Affine3d T_World_Baselink = poses_tree_.GetTransformEigen(
        poses_.GetFixedFrame(), poses_.GetMovingFrame(), m.start);

    auto l1_map = std::make_shared<PointCloud>();
    auto l2_map_init = std::make_shared<PointCloud>();
    auto l2_map_opt = std::make_shared<PointCloud>();

    pcl::transformPointCloud(
        *l1, *l1_map, T_World_Baselink * Eigen::Affine3d(config_.L1.T_B_L));
    pcl::transformPointCloud(*l2, *l2_map_init,
                             T_World_Baselink *
                                 Eigen::Affine3d(config_.L1.T_B_L) *
                                 Eigen::Affine3d(T_L1_L2_Init_));
    pcl::transformPointCloud(*l2, *l2_map_opt,
                             T_World_Baselink *
                                 Eigen::Affine3d(config_.L1.T_B_L) *
                                 Eigen::Affine3d(T_L1_L2_Opt_));

    auto l1_map_col = beam::ColorPointCloud(*l1_map, 0, 0, 255);
    auto l2_map_init_col = beam::ColorPointCloud(*l2_map_init, 255, 0, 0);
    auto l2_map_opt_col = beam::ColorPointCloud(*l2_map_opt, 0, 255, 0);

    std::string f1 =
        beam::CombinePaths(inputs_.maps_output_dir,
                           "measurement_" + std::to_string(i) + "_l1.pcd");
    std::string f2_init =
        beam::CombinePaths(inputs_.maps_output_dir,
                           "measurement_" + std::to_string(i) + "_l2_init.pcd");
    std::string f2_opt =
        beam::CombinePaths(inputs_.maps_output_dir,
                           "measurement_" + std::to_string(i) + "_l2_opt.pcd");

    beam::SavePointCloud<pcl::PointXYZRGB>(f1, l1_map_col);
    beam::SavePointCloud<pcl::PointXYZRGB>(f2_init, l2_map_init_col);
    beam::SavePointCloud<pcl::PointXYZRGB>(f2_opt, l2_map_opt_col);
  }
}
