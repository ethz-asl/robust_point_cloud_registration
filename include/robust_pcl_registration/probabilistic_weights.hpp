#ifndef POINT_CLOUD_REGISTRATION_PROBABILISTIC_WEIGHTS_H
#define POINT_CLOUD_REGISTRATION_PROBABILISTIC_WEIGHTS_H

#include <assert.h>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Sparse>

namespace point_cloud_registration {

constexpr double pi() {
  return std::atan(1) * 4;
}

class ProbabilisticWeights {
 private:
  double t_exponent_;
  double log_factor_;
  double v_;
  double log_norm_constant_;
  int dimension_;
  bool is_normal_;
  int max_neighbours_;

 public:
  ProbabilisticWeights(double v, int dimension, int max_neighbours)
    : max_neighbours_(max_neighbours), dimension_(dimension) {
    assert(dimension > 0);
    assert(v > 0.0);
    if (v < std::numeric_limits<double>::infinity()) {
      v_ = v;
      t_exponent_ = -(v + dimension_) / 2.0;
      is_normal_ = false;
      log_norm_constant_ = std::lgamma(v_ / 2) -
          std::lgamma((v_ + dimension_) / 2) +
          (v_ / 2) * std::log(pi() * v_);
    } else {
      is_normal_ = true;
      log_norm_constant_ = (dimension_ / 2.0) * std::log(2 * pi());
    }
  }

  Eigen::SparseMatrix<double, Eigen::RowMajor> updateWeights(
      Eigen::SparseMatrix<int, Eigen::RowMajor> data_association, std::vector<double> squared_errors) {
    Eigen::SparseMatrix<double, Eigen::RowMajor> weights(data_association.rows(),
                                                         data_association.cols());
    weights.reserve(Eigen::VectorXi::Constant(data_association.rows(), max_neighbours_));
    int squared_errors_index = 0;
    for (std::size_t i = 0u; i < data_association.outerSize(); ++i) {
      double max_log_prob = -std::numeric_limits<double>::infinity();
      std::vector<double> log_probs;
      std::vector<double> expected_weights;
      log_probs.reserve(max_neighbours_);
      expected_weights.reserve(max_neighbours_);
      double marginal_log_likelihood = 0;
      for (Eigen::SparseMatrix<int, Eigen::RowMajor>::InnerIterator it(data_association, i); it; ++it) {
        // Update log_probs.
        double log_prob;
        if (is_normal_) {
          log_prob = -squared_errors[squared_errors_index] / 2 + log_norm_constant_;
        } else {
          log_prob =
              (t_exponent_) * std::log1p(squared_errors[squared_errors_index] / v_) - log_norm_constant_;
          const double expected_weight = (v_ + dimension_) / (v_ + squared_errors[squared_errors_index]);
          expected_weights.push_back(expected_weight);
        }
        if (log_prob > max_log_prob) {
          max_log_prob = log_prob;
        }
        log_probs.push_back(log_prob);
        ++squared_errors_index;
      }
      for (double log_p : log_probs) {
        marginal_log_likelihood += std::exp(log_p - max_log_prob);
      }
      marginal_log_likelihood = std::log(marginal_log_likelihood) + max_log_prob;
      int k = 0;
      for (Eigen::SparseMatrix<int, Eigen::RowMajor>::InnerIterator it(data_association, i); it; ++it) {
        if (is_normal_) {
          weights.insert(it.row(), it.col()) =
              std::exp(log_probs[k] - marginal_log_likelihood);
        } else {
          weights.insert(it.row(), it.col()) =
              std::exp(log_probs[k] - marginal_log_likelihood) * expected_weights[k];
        }
        ++k;
      }
    }
    weights.makeCompressed();
    return weights;
  }
};
}  // namespace point_cloud_registration

#endif
