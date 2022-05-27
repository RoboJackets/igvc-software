cmake_minimum_required(VERSION 2.8.3)
project(igvc_navigation)

include(../igvc.cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             roscpp
             rospy
             std_msgs
             mbf_msgs
             igvc_msgs
             image_geometry
             image_transport
             cv_bridge
             tf
             tf2
             tf2_eigen
             pcl_ros
             pcl_conversions
             igvc_utils
             tf_conversions
             robot_localization
             parameter_assertions
             costmap_2d
             dynamic_reconfigure
             igvc_utils
             grid_map_core
             grid_map_ros
             grid_map_cv
             grid_map_filters
             grid_map_loader
             grid_map_msgs
             grid_map_octomap
             grid_map_rviz_plugin
             grid_map_visualization
             base_local_planner
             pluginlib
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED filters)
find_package(octomap REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  Message.msg
#)

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#  Service.srv
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
      CATKIN_DEPENDS igvc_msgs std_msgs
)

###########
## Build ##
###########

# set default build type
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
    add_compile_options(-march=native)
ENDIF (NOT CMAKE_BUILD_TYPE)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    include
)

## Declare a cpp library
# add_library(igvc
#   src/${PROJECT_NAME}/igvc.cpp
# )

## Declare a cpp executable
# add_executable(igvc_node src/igvc_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

## Specify libraries to link a library or executable target against
# target_link_libraries(igvc_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS igvc igvc_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h")
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
install( DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    #add_subdirectory(src/tests)
endif ()

# include GraphSearch header files
include_directories(GraphSearch/include)
# create GraphSearch library
add_library(
    GraphSearch STATIC
    GraphSearch/src/Node.cpp
    GraphSearch/src/Graph.cpp
    GraphSearch/src/PriorityQueue.cpp
)
add_dependencies(GraphSearch ${catkin_EXPORTED_TARGETS})
# link OpenCV and cv_bridge to GraphSearch library
target_link_libraries(GraphSearch ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_subdirectory(src/back_circle)
add_subdirectory(src/back_up_recovery)
add_subdirectory(src/swerve_drive)
add_subdirectory(src/mapper)
add_subdirectory(src/navigation_client)
add_subdirectory(src/navigation_server)
add_subdirectory(src/swerve_wheel_odometer)
add_subdirectory(src/swervi_differential_drive)
