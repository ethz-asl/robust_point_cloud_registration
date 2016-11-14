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
  bool use_gaussian;
  double radius;
  std::string source_cloud;
  std::string target_cloud;
  double source_filter_size;
  double target_filter_size;
  int dof;
};

class Icp {
 public:
    Icp(const IcpParameters& params);
    void evaluate(
        pcl::PointCloud<PointType>::Ptr source_cloud,
        pcl::PointCloud<PointType>::Ptr target_cloud,
        const Eigen::Vector3d& delta_t);
 private:
  IcpParameters params_;
};

#endif // ICP_H_
