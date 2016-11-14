#include <limits>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include "point_cloud_registration/point_cloud_registration.h"

using point_cloud_registration::PointCloudRegistration;
using point_cloud_registration::PointCloudRegistrationParams;

pcl::PointCloud<pcl::PointXYZ> generateCloud()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    int width = 30;
    int height = 50;
    cloud.reserve(width * height);
    double x = 0;
    for (std::size_t i = 0; i < width; ++i)
    {
        double y = 0;
        for (std::size_t j = 0; j < height; ++j)
        {
            cloud.push_back(pcl::PointXYZ(x, y, sin(x) + cos(y)));
            y += 0.5;
        }
        x += 0.5;
    }
    return cloud;
}

TEST(PointCloudRegistrationTestSuite, exactDataAssociationGaussianTest)
{
    auto source_cloud = generateCloud();
    pcl::PointCloud<pcl::PointXYZ> target_cloud;
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() << 2.5, 0.0, 0.0;
    transform.prerotate (Eigen::AngleAxisd (0.34, Eigen::Vector3d::UnitZ()));
    pcl::transformPointCloud(source_cloud, target_cloud, transform);
    Eigen::SparseMatrix<int, Eigen::RowMajor> data_association(source_cloud.size(), target_cloud.size());
    std::vector<Eigen::Triplet<int>> tripletList;
    for (std::size_t i = 0; i < source_cloud.size(); ++i)
    {
        tripletList.push_back(Eigen::Triplet<int>(i, i, 1));
    }
    data_association.setFromTriplets(tripletList.begin(), tripletList.end());
    data_association.makeCompressed();
    PointCloudRegistrationParams params;
    params.dof = std::numeric_limits<double>::infinity();
    params.max_neighbours = 3;
    params.dimension = 3;
    PointCloudRegistration registration(source_cloud, target_cloud, data_association, params);
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.use_nonmonotonic_steps = true;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = std::numeric_limits<int>::max();
    options.function_tolerance = 10e-5;
    options.num_threads = 8;
    ceres::Solver::Summary summary;
    registration.solve(options, &summary);

    auto estimated_transform = registration.transformation();
    pcl::PointCloud<pcl::PointXYZ> aligned_source;
    pcl::transformPointCloud (source_cloud, aligned_source, estimated_transform);
    double mean_error = 0;
    for (std::size_t i = 0; i < target_cloud.size(); ++i)
    {
        double error = std::sqrt(std::pow(target_cloud.at(i).x - aligned_source[i].x, 2) +
                                 std::pow(target_cloud.at(i).y - aligned_source[i].y, 2) +
                                 std::pow(target_cloud.at(i).z - aligned_source[i].z, 2));
        mean_error += error;
    }
    mean_error /= target_cloud.size();
    EXPECT_NEAR(mean_error, 0, 1e-6);
}

TEST(PointCloudRegistrationTestSuite, exactDataAssociationTDistributionTest)
{
    auto source_cloud = generateCloud();
    pcl::PointCloud<pcl::PointXYZ> target_cloud;
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() << 2.5, 0.0, 0.0;
    transform.prerotate (Eigen::AngleAxisd (0.34, Eigen::Vector3d::UnitZ()));
    pcl::transformPointCloud(source_cloud, target_cloud, transform);
    Eigen::SparseMatrix<int, Eigen::RowMajor> data_association(source_cloud.size(), target_cloud.size());
    std::vector<Eigen::Triplet<int>> tripletList;
    for (std::size_t i = 0; i < source_cloud.size(); ++i)
    {
        tripletList.push_back(Eigen::Triplet<int>(i, i, 1));
    }
    data_association.setFromTriplets(tripletList.begin(), tripletList.end());
    data_association.makeCompressed();
    PointCloudRegistrationParams params;
    params.dof = 5;
    params.max_neighbours = 3;
    params.dimension = 3;
    PointCloudRegistration registration(source_cloud, target_cloud, data_association, params);
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.use_nonmonotonic_steps = true;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = std::numeric_limits<int>::max();
    options.function_tolerance = 10e-5;
    options.num_threads = 8;
    ceres::Solver::Summary summary;
    registration.solve(options, &summary);

    auto estimated_transform = registration.transformation();
    pcl::PointCloud<pcl::PointXYZ> aligned_source;
    pcl::transformPointCloud (source_cloud, aligned_source, estimated_transform);
    double mean_error = 0;
    for (std::size_t i = 0; i < target_cloud.size(); ++i)
    {
        double error = std::sqrt(std::pow(target_cloud.at(i).x - aligned_source[i].x, 2) +
                                 std::pow(target_cloud.at(i).y - aligned_source[i].y, 2) +
                                 std::pow(target_cloud.at(i).z - aligned_source[i].z, 2));
        mean_error += error;
    }
    mean_error /= target_cloud.size();
    EXPECT_NEAR(mean_error, 0, 1e-6);
}

//TEST(PointCloudRegistrationTestSuite, nonExactDataAssociationTest)
//{
//    auto source_cloud = generateCloud();
//    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
////    transform.translation() << 1, 0.0, 0.0;
//    transform.prerotate (Eigen::AngleAxisd (0.10, Eigen::Vector3d::UnitZ()));
//    pcl::transformPointCloud(source_cloud, *target_cloud, transform);
//    Eigen::SparseMatrix<int, Eigen::RowMajor> data_association(source_cloud.size(), target_cloud->size());
//    std::vector<Eigen::Triplet<int>> tripletList;
//    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//    kdtree.setInputCloud(target_cloud);
//    std::vector<float> distances;
//    for (std::size_t i = 0; i < source_cloud.size(); i++)
//    {
//        std::vector<int> result;
//        kdtree.radiusSearch(source_cloud, i, 3, result, distances, 5);
//        for (int index : result)
//        {
//            tripletList.push_back(Eigen::Triplet<int>(i, index, 1));
//        }
//    }
//    data_association.setFromTriplets(tripletList.begin(), tripletList.end());
//    data_association.makeCompressed();
//    PointCloudRegistrationParams params;
//    params.dof = std::numeric_limits<double>::infinity();
//    params.max_neighbours = 100;
//    params.dimension = 3;
//    PointCloudRegistration registration(source_cloud, *target_cloud, data_association, params);
//    ceres::Solver::Options options;
//    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//    options.use_nonmonotonic_steps = true;
//    options.minimizer_progress_to_stdout = false;
//    options.max_num_iterations = std::numeric_limits<int>::max();
//    options.function_tolerance = 10e-5;
//    options.num_threads = 8;
//    ceres::Solver::Summary summary;
//    registration.solve(options, &summary);
//    auto estimated_transform = registration.transformation();
//    auto difference = (transform * estimated_transform.inverse()).matrix();
//    for (std::size_t i = 0; i < difference.rows(); ++i)
//    {
//        for (std::size_t j = 0; j < difference.cols(); ++j)
//        {
//            if (i == j)
//            {
//                EXPECT_NEAR(difference(i, j), 1, 1e-2);
//            }
//            else
//            {
//                EXPECT_NEAR(difference(i, j), 0, 1e-2);
//            }
//        }
//    }
//    pcl::PointCloud<pcl::PointXYZ> aligned_source;
//    pcl::transformPointCloud (source_cloud, aligned_source, estimated_transform);
//    double mean_error = 0;
//    for (std::size_t i = 0; i < target_cloud->size(); ++i)
//    {
//        double error = std::sqrt(std::pow(target_cloud->at(i).x - aligned_source[i].x, 2) +
//                                 std::pow(target_cloud->at(i).y - aligned_source[i].y, 2) +
//                                 std::pow(target_cloud->at(i).z - aligned_source[i].z, 2));
//        mean_error += error;
//    }
//    mean_error /= target_cloud->size();
//    std::cout << mean_error << std::endl;
//    pcl::visualization::PCLVisualizer viewer;
//    viewer.setBackgroundColor (0, 0, 0);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_handler(source_cloud.makeShared(), 255, 0, 0);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_handler(target_cloud, 0, 255, 0);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_source_handler(aligned_source.makeShared(), 0, 0, 255);
//    viewer.addPointCloud<pcl::PointXYZ> (source_cloud.makeShared(), source_cloud_handler, "source");
//    viewer.addPointCloud<pcl::PointXYZ> (target_cloud, target_cloud_handler, "target");
//    viewer.addPointCloud<pcl::PointXYZ> (aligned_source.makeShared(), aligned_source_handler, "aligned source");
//    viewer.spin();
//}
