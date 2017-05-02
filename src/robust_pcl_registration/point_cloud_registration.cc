#include "robust_pcl_registration/point_cloud_registration.h"

namespace point_cloud_registration {

PointCloudRegistration::PointCloudRegistration(
    const pcl::PointCloud<pcl::PointXYZ>& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>& target_cloud,
    const Eigen::SparseMatrix<int, Eigen::RowMajor>& data_association,
    PointCloudRegistrationParams parameters)
  : error_terms_(),
    data_association_(data_association),
    parameters_(parameters),
    weight_updater_(parameters.dof, parameters.dimension, parameters.max_neighbours) {
  std::copy(std::begin(parameters_.initial_rotation),
            std::end(parameters_.initial_rotation), std::begin(rotation_));
  std::copy(std::begin(parameters_.initial_translation),
            std::end(parameters_.initial_translation),
            std::begin(translation_));
  error_terms_.reserve(source_cloud.size());
  problem_.reset(new ceres::Problem());
  for (size_t i = 0u; i < data_association.outerSize(); i++) {
    for (Eigen::SparseMatrix<int, Eigen::RowMajor>::InnerIterator it(data_association, i); it; ++it){
      ErrorTerm* error_term = new ErrorTerm(source_cloud[it.row()], target_cloud[it.col()]);
      error_terms_.push_back(error_term);
      problem_->AddResidualBlock(
            new ceres::AutoDiffCostFunction <ErrorTerm, ErrorTerm::kResiduals, 4, 3> (error_term),
            error_term->weight(), rotation_, translation_);
    }
  }
  weight_updater_callback_.reset(new WeightUpdaterCallback(&data_association_, &parameters_, &error_terms_,
                                                           &weight_updater_, rotation_, translation_));
  (*weight_updater_callback_)(ceres::IterationSummary());
}

void PointCloudRegistration::solve(ceres::Solver::Options options,
                                   ceres::Solver::Summary* summary) {
  options.callbacks.push_back(weight_updater_callback_.get());
  options.update_state_every_iteration = true;
  ceres::Solve(options, problem_.get(), summary);
}

Eigen::Affine3d PointCloudRegistration::transformation() {
  Eigen::Affine3d affine = Eigen::Affine3d::Identity();
  Eigen::Quaternion<double> estimated_rot(rotation_[0], rotation_[1],
      rotation_[2], rotation_[3]);
  estimated_rot.normalize();
  affine.rotate(estimated_rot);
  affine.pretranslate(Eigen::Vector3d(translation_));
  return affine;
}
}  // namespace point_cloud_registration
