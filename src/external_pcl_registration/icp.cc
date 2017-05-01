#include "external_pcl_registration/icp.h"

#include <glog/logging.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>


Icp::Icp(const IcpParameters& params)
  : params_(params) {}

void Icp::evaluate(
    pcl::PointCloud<PointType>::Ptr source_cloud,
    pcl::PointCloud<PointType>::Ptr target_cloud) {
  CHECK(source_cloud);
  CHECK(target_cloud);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source_cloud);
  icp.setInputTarget(target_cloud);
  LOG(INFO) << "TransformationEpsilon: " << icp.getTransformationEpsilon();
  LOG(INFO) << "MaxCorrespondenceDistance: " << icp.getMaxCorrespondenceDistance();
  LOG(INFO) << "RANSACOutlierRejectionThreshold: " << icp.getRANSACOutlierRejectionThreshold();

  pcl::PointCloud<PointType>::Ptr aligned_source =
      boost::make_shared<pcl::PointCloud<PointType>>();
  icp.align(*aligned_source);
  LOG(INFO) << "Final transformation: " << std::endl << icp.getFinalTransformation();
  if (icp.hasConverged()) {
    LOG(INFO) << "ICP converged." << std::endl
              << "The score is " << icp.getFitnessScore();
  } else {
    LOG(INFO) << "ICP did not converge.";
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
        (new pcl::visualization::PCLVisualizer ("ICP: source(red), target(green), aligned(blue)"));
    viewer->setBackgroundColor(255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        source_cloud_handler(source_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_cloud_handler(target_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        aligned_source_handler(aligned_source, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(source_cloud, source_cloud_handler, "source");
    viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_cloud_handler, "target");
    viewer->addPointCloud<pcl::PointXYZ>(aligned_source, aligned_source_handler, "aligned source");
    viewer->spin();
  }
}

