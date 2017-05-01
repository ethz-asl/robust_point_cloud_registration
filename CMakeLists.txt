cmake_minimum_required (VERSION 2.8)
project(point_cloud_registration)

find_package(catkin_simple REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(VTK REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

cs_add_library(${PROJECT_NAME}
  src/robust_pcl_registration/point_cloud_registration.cc
  #src/robust_pcl_registration/point_cloud_registration_iteration.cc
  src/robust_pcl_registration/pda.cc
  src/external_pcl_registration/gicp.cc
  src/external_pcl_registration/icp.cc
  src/external_pcl_registration/ndt.cc)

# Pointcloud alignment using ICP.
cs_add_executable(icp src/main_icp.cc)
target_link_libraries(icp ${PROJECT_NAME})

# Pointcloud alignment using GICP.
cs_add_executable(gicp src/main_gicp.cc)
target_link_libraries(gicp ${PROJECT_NAME})

# Pointcloud alignment using NDT.
cs_add_executable(ndt src/main_ndt.cc)
target_link_libraries(ndt ${PROJECT_NAME})

# Pointcloud alignment using IPDA ("Robust ICP").
#cs_add_executable(icp src/main.cc)
#target_link_libraries(icp ${PROJECT_NAME})

# Executable to convert pointcloud .xyz to .pcd format.
cs_add_executable(xyz_to_pcd src/main_xyz_to_pcd.cc)

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

cs_install()
cs_export()


