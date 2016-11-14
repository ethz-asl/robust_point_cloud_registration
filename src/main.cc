#include <glog/logging.h>
#include <gflags/gflags.h>
#include <ros/ros.h>

#include <Eigen/Sparse>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>

#include "point_cloud_registration/pda.h"
#include "point_cloud_registration/gicp.h"
#include "point_cloud_registration/icp.h"
#include "point_cloud_registration/ndt.h"

DEFINE_int32(pda_max_neighbours, 50 ,"");
DEFINE_bool(pda_use_gaussian, false, "");
DEFINE_double(pda_radius, 10, "");
DEFINE_string(pda_source_cloud, "", "");
DEFINE_string(pda_target_cloud, "", "");
DEFINE_double(pda_source_filter_size, 5, "");
DEFINE_double(pda_target_filter_size, 0, "");
DEFINE_int32(pda_dof, 5, "");

DEFINE_double(delta_tx, 0.0, "");
DEFINE_double(delta_ty, 0.0, "");
DEFINE_double(delta_tz, 0.0, "");


void parsePdaParameters(PdaParameters* params) {
  params->max_neighbours = FLAGS_pda_max_neighbours;
  params->use_gaussian = FLAGS_pda_use_gaussian;
  params->radius = FLAGS_pda_radius;
  params->source_cloud = FLAGS_pda_source_cloud;
  params->target_cloud = FLAGS_pda_target_cloud;
}

void parseGicpParameters(GicpParameters* params) {
  params->max_neighbours = FLAGS_pda_max_neighbours;
  params->use_gaussian = FLAGS_pda_use_gaussian;
  params->radius = FLAGS_pda_radius;
  params->source_cloud = FLAGS_pda_source_cloud;
  params->target_cloud = FLAGS_pda_target_cloud;
}

void parseIcpParameters(IcpParameters* params) {
  params->max_neighbours = FLAGS_pda_max_neighbours;
  params->use_gaussian = FLAGS_pda_use_gaussian;
  params->radius = FLAGS_pda_radius;
  params->source_cloud = FLAGS_pda_source_cloud;
  params->target_cloud = FLAGS_pda_target_cloud;
}

void parseNdtParameters(NdtParameters* params) {
  params->max_neighbours = FLAGS_pda_max_neighbours;
  params->use_gaussian = FLAGS_pda_use_gaussian;
  params->radius = FLAGS_pda_radius;
  params->source_cloud = FLAGS_pda_source_cloud;
  params->target_cloud = FLAGS_pda_target_cloud;
}

void loadPointClouds(const PdaParameters& params,
                     pcl::PointCloud<PointType>::Ptr source_cloud,
                     pcl::PointCloud<PointType>::Ptr target_cloud) {
  pcl::io::loadPCDFile<PointType>(params.source_cloud, *source_cloud);
  pcl::io::loadPCDFile<PointType>(params.target_cloud, *target_cloud);
}

int main (int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "evaluation");
  ros::Time::init();

  PdaParameters pda_params;
  parsePdaParameters(&pda_params);

  pcl::PointCloud<PointType>::Ptr source_cloud =
      boost::make_shared<pcl::PointCloud<PointType> >();
  pcl::PointCloud<PointType>::Ptr target_cloud =
      boost::make_shared<pcl::PointCloud<PointType> >();
  loadPointClouds(pda_params, source_cloud, target_cloud);
  Pda pda(pda_params);
  pda.evaluate(source_cloud, target_cloud);

  {
    GicpParameters gicp_params;
    parseGicpParameters(&gicp_params);
    Gicp gicp(gicp_params);
    Eigen::Vector3d delta_t(FLAGS_delta_tx, FLAGS_delta_ty, FLAGS_delta_tz);
    gicp.evaluate(source_cloud, target_cloud, delta_t);
  }

  {
    IcpParameters icp_params;
    parseIcpParameters(&icp_params);
    Icp icp(icp_params);
    Eigen::Vector3d delta_t(FLAGS_delta_tx, FLAGS_delta_ty, FLAGS_delta_tz);
    icp.evaluate(source_cloud, target_cloud, delta_t);
  }

  {
    NdtParameters ndt_params;
    parseNdtParameters(&ndt_params);
    Ndt ndt(ndt_params);
    Eigen::Vector3d delta_t(FLAGS_delta_tx, FLAGS_delta_ty, FLAGS_delta_tz);
    ndt.evaluate(source_cloud, target_cloud, delta_t);
  }

  return 0;
}
