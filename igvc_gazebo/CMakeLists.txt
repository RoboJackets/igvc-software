cmake_minimum_required(VERSION 2.8.3)
project(igvc_gazebo)

include(../igvc.cmake)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED filters)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    pcl_conversions
    tf
    geometry_msgs
    igvc_msgs
    sensor_msgs
    laser_geometry
    pcl_ros
    igvc_utils
    cv_bridge
    image_transport
    robot_localization
    parameter_assertions
    )

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS std_msgs nav_msgs geometry_msgs igvc_msgs sensor_msgs
    DEPENDS
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
    )

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
    )

add_subdirectory(nodes/magnetometer)
add_subdirectory(nodes/control)
add_subdirectory(nodes/swerve_control)
add_subdirectory(nodes/scan_to_pointcloud)
add_subdirectory(nodes/ground_truth)
add_subdirectory(nodes/sim_color_detector)
