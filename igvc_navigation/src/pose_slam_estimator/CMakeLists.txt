find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${Eigen_INCLUDE_DIRS})

find_package(GTSAM REQUIRED)

# TODO(alescontrela): Add TBB to path.
# find_package(TBB REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} "/usr/local/include")

add_executable(PoseSlamEstimator PoseSlamEstimator.cpp main.cpp)

# target_link_libraries(PoseSlamEstimator ${catkin_LIBRARIES} ${ROS_LIBRARIES} /usr/local/lib/libgtsam.so ${TBB_LIBRARIES})
target_link_libraries(PoseSlamEstimator ${catkin_LIBRARIES} ${ROS_LIBRARIES} /usr/local/lib/libgtsam.so)
# target_include_directories(PoseSlamEstimator PUBLIC geodetic_conv.hpp)

add_dependencies(PoseSlamEstimator igvc_msgs_gencpp)

install(
    TARGETS PoseSlamEstimator
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
