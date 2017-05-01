#ifndef NDT_H_
#define NDT_H_

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointType;

struct NdtParameters {
  std::string source_cloud_filename;
  std::string target_cloud_filename;
  std::string aligned_cloud_filename;
  double source_filter_size;
  double target_filter_size;
  bool visualize_clouds;
  bool save_aligned_cloud;
  std::string frame_id;
  double transformation_epsilon;
  double step_size;
  double resolution;
  int maximum_iterations;
};

class Ndt {
 public:
    Ndt(const NdtParameters& params);
    void evaluate(
        pcl::PointCloud<PointType>::Ptr source_cloud,
        pcl::PointCloud<PointType>::Ptr target_cloud);
 private:
  NdtParameters params_;
};

#endif // NDT_H_
