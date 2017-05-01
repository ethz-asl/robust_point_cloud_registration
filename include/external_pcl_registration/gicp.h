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
  int max_neighbours;
  bool use_gaussian;
  double radius;
  std::string source_cloud;
  std::string target_cloud;
  double source_filter_size;
  double target_filter_size;
  int dof;
};

class Gicp {
 public:
    Gicp(const GicpParameters& params);
    void evaluate(
        pcl::PointCloud<PointType>::Ptr source_cloud,
        pcl::PointCloud<PointType>::Ptr target_cloud,
        const Eigen::Vector3d& delta_t);
 private:
  GicpParameters params_;
};

#endif // PDA_H_
