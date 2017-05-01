#ifndef ICP_H_
#define ICP_H_

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointType;

struct IcpParameters {
  int max_neighbours;
  double radius;
  std::string source_cloud_filename;
  std::string target_cloud_filename;
  std::string aligned_cloud_filename;
  double source_filter_size;
  double target_filter_size;
  bool visualize_clouds;
  bool save_aligned_cloud;
  std::string frame_id;
};

class Icp {
 public:
    Icp(const IcpParameters& params);
    void evaluate(
        pcl::PointCloud<PointType>::Ptr source_cloud,
        pcl::PointCloud<PointType>::Ptr target_cloud);
 private:
  IcpParameters params_;
};

#endif // ICP_H_
