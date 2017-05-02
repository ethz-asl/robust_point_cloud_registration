#ifndef PDA_H_
#define PDA_H_

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointType;

struct PdaParameters {
  bool save_aligned_cloud;
  bool solver_minimizer_progress_to_stdout;
  bool solver_use_nonmonotonic_steps;
  bool use_gaussian;
  bool visualize_clouds;
  double dof;
  double point_size_aligned_source;
  double point_size_source;
  double point_size_target;
  double radius;
  double solver_function_tolerance;
  double source_filter_size;
  double target_filter_size;
  double transformation_epsilon;
  int dimension;
  int maximum_iterations;
  int max_neighbours;
  int solver_maximum_iterations;
  int solver_num_threads;
  std::string aligned_cloud_filename;
  std::string frame_id;
  std::string source_cloud_filename;
  std::string target_cloud_filename;
};

class Pda {
 public:
    Pda(const PdaParameters& params);
    void evaluate(
        pcl::PointCloud<PointType>::Ptr source_cloud,
        pcl::PointCloud<PointType>::Ptr target_cloud);
 private:
  PdaParameters params_;
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
};
#endif // PDA_H_
