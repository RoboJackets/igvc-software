cmake_minimum_required(VERSION 2.8.3)
project(igvc_gazebo)
find_package(catkin REQUIRED)


set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

catkin_package()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED filters)
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  igvc_msgs
  laser_geometry
  pcl_ros
  igvc_utils
  robot_localization
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
   FILES_MATCHING PATTERN "*.yaml"
)

add_subdirectory(src/control)
add_subdirectory(src/scan_to_pointcloud)
add_subdirectory(src/ground_truth)
add_subdirectory(src/dc_motor_model)
add_subdirectory(src/encoders)
