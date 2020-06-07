cmake_minimum_required(VERSION 2.8.3)
project(gbplanner_trajopt)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

include_directories(
  ${Eigen_INCLUDE_DIRS}
)

cs_add_library(
  ${PROJECT_NAME}
  src/gbplanner_trajopt.cpp)

cs_install()
cs_export(LIBRARIES)