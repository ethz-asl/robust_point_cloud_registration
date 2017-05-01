#include "external_pcl_registration/gicp.h"

#include <glog/logging.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>


Gicp::Gicp(const GicpParameters& params)
  : params_(params) {}

void Gicp::evaluate(
    pcl::PointCloud<PointType>::Ptr source_cloud,
    pcl::PointCloud<PointType>::Ptr target_cloud) {
  CHECK(source_cloud);
  CHECK(target_cloud);

  pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
  gicp.setInputSource(source_cloud);
  gicp.setInputTarget(target_cloud);

  if (!params_.use_default_parameters) {
    gicp.setMaxCorrespondenceDistance(params_.maximum_correspondence_distance);
    gicp.setMaximumIterations(params_.maximum_iterations);
    gicp.setMaximumOptimizerIterations(params_.maximum_optimizer_iterations);
    gicp.setRANSACIterations(params_.ransac_iterations);
    gicp.setRANSACOutlierRejectionThreshold(params_.ransac_outlier_rejection_threshold);
    gicp.setTransformationEpsilon(params_.transformation_epsilon);
    gicp.setUseReciprocalCorrespondences(params_.use_reciprocal_correspondence);
  }
  LOG(INFO) << "MaxCorrespondenceDistance: " << gicp.getMaxCorrespondenceDistance();
  LOG(INFO) << "MaximumIterations: " << gicp.getMaximumIterations();
  LOG(INFO) << "MaximumOptimizerIterations: " << gicp.getMaximumOptimizerIterations();
  LOG(INFO) << "RANSACIterations: " << gicp.getRANSACIterations();
  LOG(INFO) << "RANSACOutlierRejectionThreshold: " << gicp.getRANSACOutlierRejectionThreshold();
  LOG(INFO) << "TransformationEpsilon: " << gicp.getTransformationEpsilon();
  LOG(INFO) << "MaxCorrespondenceDistance: " << gicp.getMaxCorrespondenceDistance();
  LOG(INFO) << "RANSACOutlierRejectionThreshold: " << gicp.getRANSACOutlierRejectionThreshold();
  LOG(INFO) << "UseReciprocalCorrespondences: " << gicp.getUseReciprocalCorrespondences();

  pcl::PointCloud<PointType>::Ptr aligned_source =
      boost::make_shared<pcl::PointCloud<PointType>>();
  gicp.align(*aligned_source);
  CHECK(aligned_source);
  LOG(INFO) << "Final transformation: " << std::endl << gicp.getFinalTransformation();
  if (gicp.hasConverged()) {
    LOG(INFO) << "GICP converged." << std::endl
              << "The score is " << gicp.getFitnessScore();
  } else {
    LOG(INFO) << "GICP did not converge.";
  }

  if (params_.save_aligned_cloud) {
    LOG(INFO) << "Saving aligned source cloud to: " << params_.aligned_cloud_filename;
    pcl::io::savePCDFile(params_.aligned_cloud_filename, *aligned_source);
  }

  if (params_.visualize_clouds) {
    source_cloud->header.frame_id = params_.frame_id;
    target_cloud->header.frame_id = params_.frame_id;
    aligned_source->header.frame_id = params_.frame_id;
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer
        (new pcl::visualization::PCLVisualizer ("GICP: source(red), target(green), aligned(blue)"));
    viewer->setBackgroundColor(255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
        source_cloud_handler(source_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
        target_cloud_handler(target_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
        aligned_source_handler(aligned_source, 0, 0, 255);
    viewer->addPointCloud<PointType>(source_cloud, source_cloud_handler, "source");
    viewer->addPointCloud<PointType>(target_cloud, target_cloud_handler, "target");
    viewer->addPointCloud<PointType>(aligned_source, aligned_source_handler, "aligned source");
    viewer->spin();
  }
}

