#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tools/utils.hpp>

#include "robust_pcl_registration/pda.h"


DEFINE_bool(ipda_save_aligned_cloud, true, "");
DEFINE_bool(ipda_solver_minimizer_progress_to_stdout, false, "");
DEFINE_bool(ipda_solver_use_nonmonotonic_steps, true, "");
DEFINE_bool(ipda_use_gaussian, true, "");
DEFINE_bool(ipda_visualize_clouds, true, "");
DEFINE_double(ipda_dof, 3.0, "");
DEFINE_double(ipda_point_size_aligned_source, 3.0, "");
DEFINE_double(ipda_point_size_source, 3.0, "");
DEFINE_double(ipda_point_size_target, 3.0, "");
DEFINE_double(ipda_radius, 10.0, "");
DEFINE_double(ipda_solver_function_tolerance, 1.0e-16, "");
DEFINE_double(ipda_source_filter_size, 5.0, "");
DEFINE_double(ipda_target_filter_size, 0.0, "");
DEFINE_double(ipda_transformation_epsilon, 1.0e-3, "");
DEFINE_int32(ipda_dimension, 3, "");
DEFINE_int32(ipda_maximum_iterations, 100, "");
DEFINE_int32(ipda_max_neighbours, 50, "");
DEFINE_int32(ipda_solver_maximum_iterations, 100, "");
DEFINE_int32(ipda_solver_num_threads, 8, "");
DEFINE_string(ipda_aligned_cloud_filename, "", "");
DEFINE_string(ipda_frame_id, "", "");
DEFINE_string(ipda_source_cloud_filename, "", "");
DEFINE_string(ipda_target_cloud_filename, "", "");

void parseIpdaParameters(IpdaParameters* params) {
  params->save_aligned_cloud = FLAGS_ipda_save_aligned_cloud;
  params->solver_minimizer_progress_to_stdout = FLAGS_ipda_solver_minimizer_progress_to_stdout;
  params->solver_use_nonmonotonic_steps = FLAGS_ipda_solver_use_nonmonotonic_steps;
  params->use_gaussian = FLAGS_ipda_use_gaussian;
  params->visualize_clouds = FLAGS_ipda_visualize_clouds;
  params->dof = FLAGS_ipda_dof;
  params->point_size_aligned_source = FLAGS_ipda_point_size_aligned_source;
  params->point_size_source = FLAGS_ipda_point_size_source;
  params->point_size_target = FLAGS_ipda_point_size_target;
  params->radius = FLAGS_ipda_radius;
  params->solver_function_tolerance = FLAGS_ipda_solver_function_tolerance;
  params->source_filter_size = FLAGS_ipda_source_filter_size;
  params->target_filter_size = FLAGS_ipda_target_filter_size;
  params->transformation_epsilon = FLAGS_ipda_transformation_epsilon;
  params->dimension = FLAGS_ipda_dimension;
  params->maximum_iterations = FLAGS_ipda_maximum_iterations;
  params->max_neighbours = FLAGS_ipda_max_neighbours;
  params->solver_maximum_iterations = FLAGS_ipda_solver_maximum_iterations;
  params->solver_num_threads = FLAGS_ipda_solver_num_threads;
  params->aligned_cloud_filename = FLAGS_ipda_aligned_cloud_filename;
  params->frame_id = FLAGS_ipda_frame_id;
  params->source_cloud_filename = FLAGS_ipda_source_cloud_filename;
  params->target_cloud_filename = FLAGS_ipda_target_cloud_filename;
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "IPDA");
  ros::Time::init();

  // Load the IPDA parameters.
  IpdaParameters ipda_params;
  parseIpdaParameters(&ipda_params);

  // Load source and target pointcloud.
  pcl::PointCloud<PointType>::Ptr source_cloud =
      boost::make_shared<pcl::PointCloud<PointType> >();
  pcl::PointCloud<PointType>::Ptr target_cloud =
      boost::make_shared<pcl::PointCloud<PointType> >();
  loadPointClouds(ipda_params.source_cloud_filename, ipda_params.target_cloud_filename,
                  source_cloud, target_cloud);

  // Run IPDA.
  Ipda ipda(ipda_params);
  ipda.evaluate(source_cloud, target_cloud);

  return 0;
}
