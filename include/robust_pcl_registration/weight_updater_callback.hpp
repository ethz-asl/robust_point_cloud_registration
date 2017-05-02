#ifndef POINT_CLOUD_REGISTRATION_WEIGHT_UPDATER_CALLBACK_H
#define POINT_CLOUD_REGISTRATION_WEIGHT_UPDATER_CALLBACK_H

#include <vector>

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Sparse>

#include "robust_pcl_registration/error_term.hpp"
#include "robust_pcl_registration/point_cloud_registration_params.hpp"

namespace point_cloud_registration {

class WeightUpdaterCallback : public ceres::IterationCallback {

 private:
  Eigen::SparseMatrix<int, Eigen::RowMajor> *data_association_;
  PointCloudRegistrationParams* params_;
  std::vector<ErrorTerm*>* error_terms_;
  ProbabilisticWeights* weight_updater_;
  double* rotation_;
  double* translation_;

 public:
  WeightUpdaterCallback(Eigen::SparseMatrix<int, Eigen::RowMajor> *data_association,
                        PointCloudRegistrationParams* params, std::vector<ErrorTerm*>* error_terms,
                        ProbabilisticWeights* weight_updater, double rotation[4], double translation[3]):
    data_association_(data_association), params_(params), error_terms_(error_terms), weight_updater_(weight_updater),
    rotation_(rotation), translation_(translation) {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    std::vector<double> squared_errors(data_association_->rows(), data_association_->cols());
    squared_errors.reserve(data_association_->cols() * params_->max_neighbours);
    for (std::size_t i = 0; i < error_terms_->size(); ++i) {
      double residual[3];
      (*(error_terms_->at(i)))(rotation_, translation_, residual);
      double squared_error = residual[0] * residual[0] + residual[1] * residual[1] + residual[2] * residual[2];
      squared_errors.push_back(squared_error);
    }
    Eigen::SparseMatrix<double, Eigen::RowMajor> weights_ =
        weight_updater_->updateWeights(*data_association_, squared_errors);
    for (int i = 0, k = 0; i < weights_.outerSize(); ++i) {
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(weights_, i); it; ++it) {
        error_terms_->at(k)->updateWeight(it.value());
        k++;
      }
    }
    return ceres::SOLVER_CONTINUE;
  }
};

}  // namespace point_cloud_registration

#endif
