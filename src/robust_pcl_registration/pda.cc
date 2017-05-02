#include "robust_pcl_registration/pda.h"

#include <glog/logging.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "robust_pcl_registration/point_cloud_registration.h"

Pda::Pda(const PdaParameters& params)
  : params_(params) {}

void Pda::evaluate(
    pcl::PointCloud<PointType>::Ptr source_cloud,
    pcl::PointCloud<PointType>::Ptr target_cloud) {
  CHECK(source_cloud);
  CHECK(target_cloud);

  Eigen::Affine3d final_transformation, previous_transformation, I_3;
  final_transformation.setIdentity();
  previous_transformation.setIdentity();
  I_3.setIdentity();

  pcl::PointCloud<PointType>::Ptr aligned_source =
      boost::make_shared<pcl::PointCloud<PointType> >();
  *aligned_source = *source_cloud;
  CHECK(aligned_source);

  if (params_.visualize_clouds) {
    source_cloud->header.frame_id = params_.frame_id;
    target_cloud->header.frame_id = params_.frame_id;
    aligned_source->header.frame_id = params_.frame_id;
    viewer_.reset(new pcl::visualization::PCLVisualizer ("IPDA: source(red), target(green), aligned(blue)"));
    viewer_->setBackgroundColor(255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
        source_cloud_handler(source_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
        target_cloud_handler(target_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
        aligned_source_handler(aligned_source, 0, 0, 255);
    viewer_->addPointCloud<PointType>(source_cloud, source_cloud_handler, "source");
    viewer_->addPointCloud<PointType>(target_cloud, target_cloud_handler, "target");
    viewer_->addPointCloud<PointType>(aligned_source, aligned_source_handler, "aligned_source");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             params_.point_size_target, "target");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             params_.point_size_source, "source");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             params_.point_size_aligned_source, "aligned_source");
    viewer_->spinOnce();
  }
  point_cloud_registration::PointCloudRegistrationParams params;
  params.dof = params_.dof;
  params.max_neighbours = params_.max_neighbours;
  params.dimension = params_.dimension;

  // Solver parameters.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.use_nonmonotonic_steps = params_.solver_use_nonmonotonic_steps;
  options.minimizer_progress_to_stdout = params_.solver_minimizer_progress_to_stdout;
  options.max_num_iterations = params_.solver_maximum_iterations;
  options.function_tolerance = params_.solver_function_tolerance;
  options.num_threads = params_.solver_num_threads;
  ceres::Solver::Summary summary;

  for (size_t iter = 0u; iter < params_.maximum_iterations; ++iter) {
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(target_cloud);
    std::vector<float> distances;
    Eigen::SparseMatrix<int, Eigen::RowMajor> data_association(aligned_source->size(),
                                                               target_cloud->size());
    std::vector<Eigen::Triplet<int> > tripletList;
    for (std::size_t i = 0u; i < aligned_source->size(); i++) {
      std::vector<int> neighbours;
      kdtree.radiusSearch((*aligned_source)[i],
                          params_.radius, neighbours, distances, params_.max_neighbours);
      for (int j : neighbours) {
        tripletList.push_back(Eigen::Triplet<int>(i, j, 1));
      }
    }
    data_association.setFromTriplets(tripletList.begin(), tripletList.end());
    data_association.makeCompressed();

    point_cloud_registration::PointCloudRegistration registration(
          *aligned_source, *target_cloud, data_association, params);
    registration.solve(options, &summary);
    VLOG(100) << summary.FullReport();

    const Eigen::Affine3d current_transformation = registration.transformation();
    final_transformation = current_transformation * final_transformation;
    LOG(INFO) << "Current Transformation: " << std::endl << final_transformation.matrix();
    pcl::transformPointCloud(*aligned_source, *aligned_source, current_transformation);

    if (params_.visualize_clouds) {
      source_cloud->header.frame_id = params_.frame_id;
      target_cloud->header.frame_id = params_.frame_id;
      aligned_source->header.frame_id = params_.frame_id;
      pcl::visualization::PointCloudColorHandlerCustom<PointType>
          source_cloud_handler(source_cloud, 255, 0, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointType>
          target_cloud_handler(target_cloud, 0, 255, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointType>
          aligned_source_handler(aligned_source, 0, 0, 255);
      viewer_->updatePointCloud(source_cloud, source_cloud_handler, "source");
      viewer_->updatePointCloud(target_cloud, target_cloud_handler, "target");
      viewer_->updatePointCloud(aligned_source, aligned_source_handler, "aligned_source");
      viewer_->spinOnce();
    }

    // Check convergence.
    const double transformation_epsilon =
        ((current_transformation * previous_transformation.inverse()).matrix()
         - Eigen::Matrix4d::Identity()).norm();
    LOG(INFO) << "Transformation epsilon: " << transformation_epsilon;
    if (transformation_epsilon < params_.transformation_epsilon) {
      LOG(INFO) << "IPDA converged." << std::endl;
      return;
    }
    previous_transformation = current_transformation;
  }
}
