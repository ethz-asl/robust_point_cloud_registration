#ifndef TOOLS_UTILS_HPP_
#define TOOLS_UTILS_HPP_

#include <string>

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointType;

void loadPointClouds(const std::string source_cloud_filename,
                     const std::string target_cloud_filename,
                     pcl::PointCloud<PointType>::Ptr source_cloud,
                     pcl::PointCloud<PointType>::Ptr target_cloud) {
  LOG(INFO) << "Loading source cloud: " << source_cloud_filename;
  pcl::io::loadPCDFile<PointType>(source_cloud_filename, *source_cloud);
  CHECK(source_cloud);
  CHECK(source_cloud->size() > 0u);

  LOG(INFO) << "Loading target cloud: " << target_cloud_filename;
  pcl::io::loadPCDFile<PointType>(target_cloud_filename, *target_cloud);
  CHECK(target_cloud);
  CHECK(target_cloud->size() > 0u);
}

#endif // TOOLS_UTILS_HPP_
