add_executable(imu_remap main.cpp)
add_dependencies(imu_remap igvc_msgs_gencpp)
target_link_libraries(imu_remap ${catkin_LIBRARIES})
