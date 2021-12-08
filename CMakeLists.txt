cmake_minimum_required(VERSION 3.0.2)
project(simple_splines)


add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    angles
    base_local_planner
    cmake_modules
    costmap_2d
    dynamic_reconfigure
    nav_core
    nav_msgs
    pluginlib
    sensor_msgs
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
)


find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES simple_splines
  CATKIN_DEPENDS
      base_local_planner
      dynamic_reconfigure
      nav_msgs
      pluginlib
      sensor_msgs
      roscpp
      tf2
      tf2_ros
)

add_executable(simple_splines  src/simple_splines.cpp)
target_link_libraries(simple_splines ${catkin_LIBRARIES})

