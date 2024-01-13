#pragma once

#include <cstdio>

#include <beam_utils/math.h>
#include <beam_utils/se3.h>

#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/rotation.h>

/**
 * @brief Ceres cost functor for a prior pose with a weighting matrix (typically
 * sqrt inverse covariance)
 */
class CeresPosePriorCostFunctor {
public:
  /**
   * @brief Constructor
   * @param T Prior pose estimate
   * @param A residual weighting matrix on the pose estimate (6x6)
   */
  CeresPosePriorCostFunctor(const Eigen::Matrix4d& T) {
    beam::TransformMatrixToQuaternionAndTranslation(T, q_, p_);
  }

  template <typename T>
  bool operator()(const T* const p_wxyz, const T* const q_wxyz,
                  T* residuals) const {
    // Compute the delta quaternion
    T orientation[4] = {q_wxyz[0], q_wxyz[1], q_wxyz[2], q_wxyz[3]};
    T observation_inverse[4] = {T(q_.w()), T(-q_.x()), T(-q_.y()), T(-q_.z())};
    T difference[4];
    ceres::QuaternionProduct(observation_inverse, orientation, difference);
    ceres::QuaternionToAngleAxis(difference, residuals);

    // Compute the position error
    residuals[3] = p_wxyz[0] - T(p_(0));
    residuals[4] = p_wxyz[1] - T(p_(1));
    residuals[5] = p_wxyz[2] - T(p_(2));
    return true;
  }

  // Factory to hide the construction of the CostFunctor object from
  // the client code.
  static ceres::CostFunction* Create(const Eigen::Matrix4d& T) {
    return (new ceres::AutoDiffCostFunction<CeresPosePriorCostFunctor, 6, 3, 4>(
        new CeresPosePriorCostFunctor(T)));
  }

  Eigen::Vector3d p_;
  Eigen::Quaterniond q_;
};
