#ifndef GICP_H_
#define GICP_H_

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointType;

struct GicpParameters {
  std::string source_cloud_filename;
  std::string target_cloud_filename;
  std::string aligned_cloud_filename;
  double source_filter_size;
  double target_filter_size;
  bool visualize_clouds;
  bool save_aligned_cloud;
  std::string frame_id;
  double transformation_epsilon;
  int maximum_iterations;
  int maximum_optimizer_iterations;
  double maximum_correspondence_distance;
  int ransac_iterations;
  double ransac_outlier_rejection_threshold;
  bool use_reciprocal_correspondence;
  bool use_default_parameters;
};

class Gicp {
 public:
    Gicp(const GicpParameters& params);
    void evaluate(
        pcl::PointCloud<PointType>::Ptr source_cloud,
        pcl::PointCloud<PointType>::Ptr target_cloud);
 private:
  GicpParameters params_;
};

#endif // GICP_H_
