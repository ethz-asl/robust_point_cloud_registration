#include "robust_pcl_registration/pda.h"

#include <glog/logging.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "robust_pcl_registration/point_cloud_registration.h"
//#include "prob_point_cloud_registration/point_cloud_registration_iteration.h"

Pda::Pda(const PdaParameters& params)
  : params_(params) {}

void Pda::evaluate(
    pcl::PointCloud<PointType>::Ptr source_cloud,
    pcl::PointCloud<PointType>::Ptr target_cloud,
    double translation) {
  VLOG(1) << "Evaluating PDA";
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translation() << translation, 0.0, 0.0;
  pcl::transformPointCloud(*source_cloud, *source_cloud, transform);

//    pcl::PointCloud<PointType>::Ptr filtered_source_cloud =
//        boost::make_shared<pcl::PointCloud<PointType>>();
//    if (params_.source_filter_size > 0) {
//    pcl::VoxelGrid<PointType> filter;
//    filter.setInputCloud(source_cloud);
//    filter.setLeafSize(params_.source_filter_size,
//                       params_.source_filter_size,
//                       params_.source_filter_size);
//    filter.filter(*filtered_source_cloud);
//    }
//    *source_cloud = *filtered_source_cloud;

  // Set Kd Tree
  VLOG(1) << "KdTree";
  pcl::KdTreeFLANN<PointType> kdtree;
  kdtree.setInputCloud(target_cloud);
  std::vector<float> distances;
  Eigen::SparseMatrix<int, Eigen::RowMajor> data_association(source_cloud->size(),
                                                             target_cloud->size());
  std::vector<Eigen::Triplet<int> > tripletList;
  for (std::size_t i = 0u; i < source_cloud->size(); i++) {
    std::vector<int> neighbours;
    kdtree.radiusSearch(*source_cloud, i,
                        params_.radius, neighbours, distances,
                        params_.max_neighbours);
    for (int j : neighbours) {
      tripletList.push_back(Eigen::Triplet<int>(i, j, 1));
    }
  }
  data_association.setFromTriplets(tripletList.begin(), tripletList.end());
  data_association.makeCompressed();

  point_cloud_registration::PointCloudRegistrationParams params;
  VLOG(1) << "Running optimization";
//  prob_point_cloud_registration::PointCloudRegistrationParams params;
  params.dof = std::numeric_limits<double>::infinity();
  params.max_neighbours = params_.max_neighbours;
  params.dimension = 3;
//  prob_point_cloud_registration::PointCloudRegistrationIteration
//      registration(*source_cloud, *target_cloud, data_association, params);

  point_cloud_registration::PointCloudRegistration registration(
        *source_cloud, *target_cloud, data_association, params);
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.use_nonmonotonic_steps = true;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 100;//std::numeric_limits<int>::max();
  options.function_tolerance = 10e-16;
  options.num_threads = 8;
  ceres::Solver::Summary summary;
  VLOG(1) << "Solve.";
  registration.solve(options, &summary);

  VLOG(3) << summary.FullReport();

  auto estimated_transform = registration.transformation();
  std::cout << "estimated_transform = " << estimated_transform.translation()
            << std::endl;


  //std::string aligned_source_name = "aligned_" + source_file_name;
  //ROS_INFO("Saving aligned source cloud to: %s", aligned_source_name.c_str());
  // pcl::io::savePCDFile(aligned_source_name, *aligned_source);

  pcl::PointCloud<PointType>::Ptr aligned_source =
      boost::make_shared<pcl::PointCloud<PointType>>();
  pcl::transformPointCloud (*source_cloud, *aligned_source, estimated_transform);
  source_cloud->header.frame_id = "map";
  target_cloud->header.frame_id = "map";
  aligned_source->header.frame_id = "map";
{
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer
      (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //pcl::visualization::PCLVisualizer viewer;
  viewer->setBackgroundColor (255, 255, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      source_cloud_handler(source_cloud, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      target_cloud_handler(target_cloud, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      aligned_source_handler(aligned_source, 0, 0, 255);
  viewer->addPointCloud<pcl::PointXYZ>(source_cloud, source_cloud_handler, "source");
  viewer->addPointCloud<pcl::PointXYZ> (target_cloud, target_cloud_handler, "target");
  //viewer->addPointCloud<pcl::PointXYZ> (aligned_source, aligned_source_handler, "aligned source");
  viewer->spin();
  }

  {
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer
        (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //pcl::visualization::PCLVisualizer viewer;
    viewer->setBackgroundColor (255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        source_cloud_handler(source_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_cloud_handler(target_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        aligned_source_handler(aligned_source, 0, 0, 255);
    //viewer->addPointCloud<pcl::PointXYZ>(source_cloud, source_cloud_handler, "source");
    viewer->addPointCloud<pcl::PointXYZ> (target_cloud, target_cloud_handler, "target");
    viewer->addPointCloud<pcl::PointXYZ> (aligned_source, aligned_source_handler, "aligned source");
    viewer->spin();
  }
}
