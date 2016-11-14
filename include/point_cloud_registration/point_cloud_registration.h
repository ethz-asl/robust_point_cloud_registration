#ifndef POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_H
#define POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_H

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

#include "point_cloud_registration/error_term.hpp"
#include "point_cloud_registration/probabilistic_weights.hpp"
#include "point_cloud_registration/weight_updater_callback.hpp"
#include "point_cloud_registration/point_cloud_registration_params.hpp"

namespace point_cloud_registration
{

class PointCloudRegistration
{
public:
    PointCloudRegistration(
        const pcl::PointCloud<pcl::PointXYZ> &source_cloud,
        const pcl::PointCloud<pcl::PointXYZ> &target_cloud,
        const Eigen::SparseMatrix<int, Eigen::RowMajor> &data_association,
        PointCloudRegistrationParams parameters);
    void solve(ceres::Solver::Options options, ceres::Solver::Summary *Summary);
    Eigen::Affine3d transformation();

private:
    std::vector<ErrorTerm*> error_terms_;
    std::unique_ptr<ceres::Problem> problem_;
    double rotation_[4];
    double translation_[3];
    Eigen::SparseMatrix<int, Eigen::RowMajor> data_association_;
    ProbabilisticWeights weight_updater_;
    std::unique_ptr<WeightUpdaterCallback> weight_updater_callback_;
    PointCloudRegistrationParams parameters_;
};

}  // namespace point_cloud_registration

#endif
