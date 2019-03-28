add_executable(imu YostLabDriver.cpp yostlab_driver_node.cpp)
add_dependencies(imu igvc_msgs_gencpp)
target_link_libraries(imu ${catkin_LIBRARIES})
