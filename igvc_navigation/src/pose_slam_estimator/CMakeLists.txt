find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${Eigen_INCLUDE_DIRS})

find_package(GTSAM REQUIRED)

# Build TBB.
include(/home/alejandro/tbb/cmake/TBBGet.cmake)
tbb_get(TBB_ROOT tbb_root CONFIG_DIR TBB_DIR)
find_package(TBB REQUIRED)

find_package (GeographicLib REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} "/usr/local/include")

add_executable(pose_slam_estimator PoseSlamEstimator.cpp main.cpp)

target_link_libraries(pose_slam_estimator ${catkin_LIBRARIES} ${ROS_LIBRARIES} /usr/local/lib/libgtsam.so ${TBB_IMPORTED_TARGETS} ${GeographicLib_LIBRARIES})
# target_include_directories(PoseSlamEstimator PUBLIC geodetic_conv.hpp)

add_dependencies(pose_slam_estimator igvc_msgs_gencpp)

install(
    TARGETS pose_slam_estimator
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
