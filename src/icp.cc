#include "point_cloud_registration/icp.h"

#include <glog/logging.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>

typedef pcl::PointXYZ PointType;

Icp::Icp(const IcpParameters& params)
  : params_(params) {}

void Icp::evaluate(
    pcl::PointCloud<PointType>::Ptr source_cloud,
    pcl::PointCloud<PointType>::Ptr target_cloud,
    const Eigen::Vector3d& translation) {
  VLOG(1) << "Evaluating ICP";
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translation() << translation;
  pcl::transformPointCloud(*source_cloud, *source_cloud, transform);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source_cloud);
  icp.setInputTarget(target_cloud);
  pcl::PointCloud<PointType>::Ptr aligned_source =
      boost::make_shared<pcl::PointCloud<PointType>>();
  icp.align(*aligned_source);
  std::cout << "icp.getFinalTransformation() = "
            << icp.getFinalTransformation() << std::endl;
  if (icp.hasConverged()) {
    std::cout << "ICP converged." << std::endl
              << "The score is " << icp.getFitnessScore() << std::endl;
    std::cout << "Transformation matrix:" << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
  }
  else std::cout << "ICP did not converge." << std::endl;
  pcl::transformPointCloud(*source_cloud, *aligned_source, icp.getFinalTransformation());


  //  auto estimated_transform = registration.transformation();
  //  std::cout << "estimated_transform = " << estimated_transform.translation()
  //            << std::endl;


  //std::string aligned_source_name = "aligned_" + source_file_name;
  //ROS_INFO("Saving aligned source cloud to: %s", aligned_source_name.c_str());
  // pcl::io::savePCDFile(aligned_source_name, *aligned_source);

  //  pcl::PointCloud<PointType>::Ptr aligned_source =
  //      boost::make_shared<pcl::PointCloud<PointType>>();
  //pcl::transformPointCloud (*source_cloud, *aligned_source, estimated_transform);
  source_cloud->header.frame_id = "map";
  target_cloud->header.frame_id = "map";
  aligned_source->header.frame_id = "map";
  {
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer
        (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //pcl::visualization::PCLVisualizer viewer;
    viewer->setBackgroundColor (255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        source_cloud_handler(source_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_cloud_handler(target_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        aligned_source_handler(aligned_source, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(source_cloud, source_cloud_handler, "source");
    viewer->addPointCloud<pcl::PointXYZ> (target_cloud, target_cloud_handler, "target");
    //viewer->addPointCloud<pcl::PointXYZ> (aligned_source, aligned_source_handler, "aligned source");
    viewer->spin();
  }

  {
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer
        (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //pcl::visualization::PCLVisualizer viewer;
    viewer->setBackgroundColor (255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        source_cloud_handler(source_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_cloud_handler(target_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        aligned_source_handler(aligned_source, 0, 0, 255);
    //viewer->addPointCloud<pcl::PointXYZ>(source_cloud, source_cloud_handler, "source");
    viewer->addPointCloud<pcl::PointXYZ> (target_cloud, target_cloud_handler, "target");
    viewer->addPointCloud<pcl::PointXYZ> (aligned_source, aligned_source_handler, "aligned source");
    viewer->spin();
  }
}

