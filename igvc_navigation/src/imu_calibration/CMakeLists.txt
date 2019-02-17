add_executable(imu_calibrate imu_calibration.h imu_calibration.cpp)
add_dependencies(imu_calibrate igvc_msgs_gencpp)
target_link_libraries(imu_calibrate ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN_LIBRARIES} ${BOOST_LIBRARIES})
