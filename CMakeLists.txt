cmake_minimum_required (VERSION 2.8)
project(point_cloud_registration)

find_package(catkin_simple REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(VTK REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

cs_add_library(${PROJECT_NAME}
  src/point_cloud_registration.cc
  include/point_cloud_registration/weight_updater_callback.hpp
  include/point_cloud_registration/point_cloud_registration_params.hpp
  include/point_cloud_registration/probabilistic_weights.hpp
  include/point_cloud_registration/error_term.hpp
  src/point_cloud_registration_iteration.cc
  include/prob_point_cloud_registration/weight_updater_callback.hpp
  include/prob_point_cloud_registration/point_cloud_registration_params.hpp
  include/prob_point_cloud_registration/probabilistic_weights.hpp
  include/prob_point_cloud_registration/error_term.hpp
  src/pda.cc
  include/point_cloud_registration/pda.h
  src/gicp.cc
  include/point_cloud_registration/gicp.h
  src/icp.cc
  include/point_cloud_registration/icp.h
  src/ndt.cc
  include/point_cloud_registration/ndt.h)

cs_add_executable(pda src/main.cc)

cs_install()
cs_export()
