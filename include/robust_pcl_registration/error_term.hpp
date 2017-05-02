#ifndef INCLUDE_POINT_CLOUD_REGISTRATION_REPROJECTION_ERROR_H
#define INCLUDE_POINT_CLOUD_REGISTRATION_REPROJECTION_ERROR_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <pcl/point_types.h>

namespace point_cloud_registration {
class ErrorTerm {
 public:
  static const int kResiduals = 3;
  ErrorTerm(const pcl::PointXYZ source_point,
            const pcl::PointXYZ target_point)
      : source_point_(source_point.x, source_point.y, source_point.z),
        target_point_(target_point.x, target_point.y, target_point.z),
        weight_(new ceres::LossFunctionWrapper(
            new ceres::ScaledLoss(NULL, 1, ceres::TAKE_OWNERSHIP),
            ceres::TAKE_OWNERSHIP)){}

  template <typename T>
  bool operator()(const T* const rotation, const T* const translation,
                  T* residuals) const {
    T point_x[kResiduals];
    T point_y[kResiduals];
    for (int i = 0; i < kResiduals; i++) {
      point_x[i] = T(source_point_[i]);
      point_y[i] = T(target_point_[i]);
    }
    T transformed_point[kResiduals];
    ceres::QuaternionRotatePoint(rotation, point_x, transformed_point);
    for (int i = 0; i < kResiduals; i++) {
      transformed_point[i] = transformed_point[i] + translation[i];
      residuals[i] = point_y[i] - transformed_point[i];
    }
    return true;
  }

  void updateWeight(double new_weight) {
    weight_->Reset(
        new ceres::ScaledLoss(NULL, new_weight, ceres::TAKE_OWNERSHIP),
        ceres::TAKE_OWNERSHIP);
  }

  ceres::LossFunctionWrapper* weight() { return weight_; }

 private:
  Eigen::Vector3d source_point_;
  Eigen::Vector3d target_point_;
  ceres::LossFunctionWrapper* weight_;
};
}  // namespace point_cloud_registration
#endif  // INCLUDE_POINT_CLOUD_REGISTRATION_REPROJECTION_ERROR_H_
