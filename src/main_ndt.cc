#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tools/utils.hpp>

#include "external_pcl_registration/ndt.h"


DEFINE_string(ndt_source_cloud_filename, "", "");
DEFINE_string(ndt_target_cloud_filename, "", "");
DEFINE_string(ndt_aligned_cloud_filename, "", "");
DEFINE_double(ndt_source_filter_size, 5, "");
DEFINE_double(ndt_target_filter_size, 0, "");
DEFINE_bool(ndt_save_aligned_cloud, true, "");
DEFINE_bool(ndt_visualize_clouds, true, "");
DEFINE_string(ndt_frame_id, "", "");
DEFINE_double(ndt_transformation_epsilon, 0.01, "");
DEFINE_double(ndt_step_size, 0.1, "");
DEFINE_double(ndt_resolution, 1.0, "");
DEFINE_int32(ndt_maximum_iterations, 100, "");
DEFINE_bool(ndt_use_default_parameters, true, "");

void parseNdtParameters(NdtParameters* params) {
  params->source_cloud_filename = FLAGS_ndt_source_cloud_filename;
  params->target_cloud_filename = FLAGS_ndt_target_cloud_filename;
  params->aligned_cloud_filename = FLAGS_ndt_aligned_cloud_filename;
  params->visualize_clouds = FLAGS_ndt_visualize_clouds;
  params->save_aligned_cloud = FLAGS_ndt_save_aligned_cloud;
  params->frame_id = FLAGS_ndt_frame_id;
  params->transformation_epsilon = FLAGS_ndt_transformation_epsilon;
  params->step_size = FLAGS_ndt_step_size;
  params->resolution = FLAGS_ndt_resolution;
  params->maximum_iterations = FLAGS_ndt_maximum_iterations;
  params->use_default_parameters = FLAGS_ndt_use_default_parameters;
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "NDT");
  ros::Time::init();

  // Load the NDT parameters.
  NdtParameters ndt_params;
  parseNdtParameters(&ndt_params);

  // Load source and target pointcloud.
  pcl::PointCloud<PointType>::Ptr source_cloud =
      boost::make_shared<pcl::PointCloud<PointType> >();
  pcl::PointCloud<PointType>::Ptr target_cloud =
      boost::make_shared<pcl::PointCloud<PointType> >();
  loadPointClouds(ndt_params.source_cloud_filename, ndt_params.target_cloud_filename,
                  source_cloud, target_cloud);

  // Run NDT.
  Ndt ndt(ndt_params);
  ndt.evaluate(source_cloud, target_cloud);

  return 0;
}
