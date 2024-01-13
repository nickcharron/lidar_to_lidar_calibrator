#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>

#include <beam_optimization/CeresParams.h>
#include <beam_utils/se3.h>

#include <lidar_to_lidar_calibrator/ceres_pose_prior_cost_functor.h>

TEST_CASE("Testing Ceres Cost Functions - Perfect Initials") {
  // generate calibrations and measurements
  Eigen::Matrix4d T_B_L1 = Eigen::Matrix4d::Identity();
  Eigen::VectorXd pert(6);
  pert << 90, 30, 10, 0.1, 0.05, -0.05;
  Eigen::Matrix4d T_B_L2 = beam::PerturbTransformDegM(T_B_L1, pert);
  Eigen::Matrix4d T_L1_L2 = beam::InvertTransform(T_B_L1) * T_B_L2;

  // setup problem & params
  beam_optimization::CeresParams ceres_params;
  ceres_params.GetSolverOptionsMutable().minimizer_progress_to_stdout = true;
  std::shared_ptr<ceres::Problem> problem =
      std::make_shared<ceres::Problem>(ceres_params.ProblemOptions());
  std::unique_ptr<ceres::LossFunction> loss_function =
      ceres_params.LossFunction();

  // add calibration
  Eigen::Matrix3d R = T_L1_L2.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  Eigen::Vector3d t = T_L1_L2.block(0, 3, 3, 1);
  std::vector<double> p{t[0], t[1], t[2]};
  std::vector<double> o{q.w(), q.x(), q.y(), q.z()};

  std::unique_ptr<ceres::LocalParameterization> quat_parameterization(
      new ceres::QuaternionParameterization());
  std::unique_ptr<ceres::LocalParameterization> identity_parameterization(
      new ceres::IdentityParameterization(3));

  problem->AddParameterBlock(&(p[0]), 3, identity_parameterization.get());
  problem->AddParameterBlock(&(o[0]), 4, quat_parameterization.get());

  // add measurement
  for (int i = 0; i < 1; i++) {
    const auto& T_m = T_L1_L2;
    std::unique_ptr<ceres::CostFunction> cost(
        CeresPosePriorCostFunctor::Create(T_m));
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
  Eigen::Matrix4d T_L1_L2_Opt;
  T_L1_L2_Opt.setIdentity();
  T_L1_L2_Opt.block(0, 0, 3, 3) = R_opt;
  T_L1_L2_Opt(0, 3) = p[0];
  T_L1_L2_Opt(1, 3) = p[1];
  T_L1_L2_Opt(2, 3) = p[2];

  std::cout << "T_L1_L2:\n" << T_L1_L2 << "\n";
  std::cout << "T_L1_L2_Opt:\n" << T_L1_L2_Opt << "\n";

  REQUIRE(T_L1_L2.norm() == T_L1_L2_Opt.norm());
}

TEST_CASE("Testing Ceres Cost Functions - Perturbed Initials") {
  // generate calibrations and measurements
  Eigen::Matrix4d T_B_L1 = Eigen::Matrix4d::Identity();
  Eigen::VectorXd pert1(6);
  pert1 << 90, 30, 10, 0.1, 0.05, -0.05;
  Eigen::Matrix4d T_B_L2 = beam::PerturbTransformDegM(T_B_L1, pert1);
  Eigen::Matrix4d T_L1_L2 = beam::InvertTransform(T_B_L1) * T_B_L2;

  Eigen::VectorXd pert2(6);
  pert2 << 5, -10, 12, 0.01, -0.03, 0.02;
  Eigen::Matrix4d T_L1_L2_Init = beam::PerturbTransformDegM(T_L1_L2, pert2);

  // setup problem & params
  beam_optimization::CeresParams ceres_params;
  ceres_params.GetSolverOptionsMutable().minimizer_progress_to_stdout = true;
  std::shared_ptr<ceres::Problem> problem =
      std::make_shared<ceres::Problem>(ceres_params.ProblemOptions());
  std::unique_ptr<ceres::LossFunction> loss_function =
      ceres_params.LossFunction();

  // add calibration
  Eigen::Matrix3d R = T_L1_L2_Init.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  Eigen::Vector3d t = T_L1_L2_Init.block(0, 3, 3, 1);
  std::vector<double> p{t[0], t[1], t[2]};
  std::vector<double> o{q.w(), q.x(), q.y(), q.z()};

  std::unique_ptr<ceres::LocalParameterization> quat_parameterization(
      new ceres::QuaternionParameterization());
  std::unique_ptr<ceres::LocalParameterization> identity_parameterization(
      new ceres::IdentityParameterization(3));

  problem->AddParameterBlock(&(p[0]), 3, identity_parameterization.get());
  problem->AddParameterBlock(&(o[0]), 4, quat_parameterization.get());

  // add measurement
  for (int i = 0; i < 1; i++) {
    const auto& T_m = T_L1_L2;
    std::unique_ptr<ceres::CostFunction> cost(
        CeresPosePriorCostFunctor::Create(T_m));
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
  Eigen::Matrix4d T_L1_L2_Opt;
  T_L1_L2_Opt.setIdentity();
  T_L1_L2_Opt.block(0, 0, 3, 3) = R_opt;
  T_L1_L2_Opt(0, 3) = p[0];
  T_L1_L2_Opt(1, 3) = p[1];
  T_L1_L2_Opt(2, 3) = p[2];

  std::cout << "T_L1_L2:\n" << T_L1_L2 << "\n";
  std::cout << "T_L1_L2_Init:\n" << T_L1_L2_Init << "\n";
  std::cout << "T_L1_L2_Opt:\n" << T_L1_L2_Opt << "\n";

  REQUIRE(std::abs(T_L1_L2.norm() - T_L1_L2_Opt.norm()) < 1e-5);
}

TEST_CASE("Testing Ceres Cost Functions - Perturbed Initials & Measurements") {
  // generate calibrations and measurements
  Eigen::Matrix4d T_B_L1 = Eigen::Matrix4d::Identity();
  Eigen::VectorXd pert1(6);
  pert1 << 90, 30, 10, 0.1, 0.05, -0.05;
  Eigen::Matrix4d T_B_L2 = beam::PerturbTransformDegM(T_B_L1, pert1);
  Eigen::Matrix4d T_L1_L2 = beam::InvertTransform(T_B_L1) * T_B_L2;

  Eigen::VectorXd pert2(6);
  pert2 << 5, -10, 12, 0.01, -0.03, 0.02;
  Eigen::Matrix4d T_L1_L2_Init = beam::PerturbTransformDegM(T_L1_L2, pert2);

  // setup problem & params
  beam_optimization::CeresParams ceres_params;
  ceres_params.GetSolverOptionsMutable().minimizer_progress_to_stdout = true;
  std::shared_ptr<ceres::Problem> problem =
      std::make_shared<ceres::Problem>(ceres_params.ProblemOptions());
  std::unique_ptr<ceres::LossFunction> loss_function =
      ceres_params.LossFunction();

  // add calibration
  Eigen::Matrix3d R = T_L1_L2_Init.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  Eigen::Vector3d t = T_L1_L2_Init.block(0, 3, 3, 1);
  std::vector<double> p{t[0], t[1], t[2]};
  std::vector<double> o{q.w(), q.x(), q.y(), q.z()};

  std::unique_ptr<ceres::LocalParameterization> quat_parameterization(
      new ceres::QuaternionParameterization());
  std::unique_ptr<ceres::LocalParameterization> identity_parameterization(
      new ceres::IdentityParameterization(3));

  problem->AddParameterBlock(&(p[0]), 3, identity_parameterization.get());
  problem->AddParameterBlock(&(o[0]), 4, quat_parameterization.get());

  // add measurement
  const float max_pert_rot = 3;
  const float min_pert_rot = -3;
  const float max_pert_trans = 0.03;
  const float min_pert_trans = -0.03;
  for (int i = 0; i < 100; i++) {
    Eigen::VectorXd m_pert(6);
    m_pert << beam::randf(max_pert_rot, min_pert_rot),
        beam::randf(max_pert_rot, min_pert_rot),
        beam::randf(max_pert_rot, min_pert_rot),
        beam::randf(max_pert_trans, min_pert_trans),
        beam::randf(max_pert_trans, min_pert_trans),
        beam::randf(max_pert_trans, min_pert_trans);
    const auto T_m = beam::PerturbTransformDegM(T_L1_L2, m_pert);
    std::unique_ptr<ceres::CostFunction> cost(
        CeresPosePriorCostFunctor::Create(T_m));
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
  Eigen::Matrix4d T_L1_L2_Opt;
  T_L1_L2_Opt.setIdentity();
  T_L1_L2_Opt.block(0, 0, 3, 3) = R_opt;
  T_L1_L2_Opt(0, 3) = p[0];
  T_L1_L2_Opt(1, 3) = p[1];
  T_L1_L2_Opt(2, 3) = p[2];

  std::cout << "T_L1_L2:\n" << T_L1_L2 << "\n";
  std::cout << "T_L1_L2_Init:\n" << T_L1_L2_Init << "\n";
  std::cout << "T_L1_L2_Opt:\n" << T_L1_L2_Opt << "\n";

  REQUIRE(std::abs(T_L1_L2.norm() - T_L1_L2_Opt.norm()) < 1e-3);
}
