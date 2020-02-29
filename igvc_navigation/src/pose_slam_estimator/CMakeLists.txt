find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${Eigen_INCLUDE_DIRS})

find_package(GTSAM REQUIRED)

# TODO(alescontrela): Add TBB to path.
# include(/home/alejandro/tbb/cmake/TBBBuild.cmake)

# Build TBB.
# tbb_build(TBB_ROOT ${TBB_ROOT} CONFIG_DIR TBB_DIR MAKE_ARGS tbb_cpf=0)

# find_package(TBB REQUIRED)
include(/home/alejandro/tbb/cmake/TBBGet.cmake)
tbb_get(TBB_ROOT tbb_root CONFIG_DIR TBB_DIR)
find_package(TBB REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} "/usr/local/include")

add_executable(pose_slam_estimator PoseSlamEstimator.cpp main.cpp)

# target_link_libraries(PoseSlamEstimator ${catkin_LIBRARIES} ${ROS_LIBRARIES} /usr/local/lib/libgtsam.so ${TBB_LIBRARIES})
target_link_libraries(pose_slam_estimator ${catkin_LIBRARIES} ${ROS_LIBRARIES} /usr/local/lib/libgtsam.so ${TBB_IMPORTED_TARGETS})
# target_include_directories(PoseSlamEstimator PUBLIC geodetic_conv.hpp)

add_dependencies(pose_slam_estimator igvc_msgs_gencpp)

install(
    TARGETS pose_slam_estimator
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
