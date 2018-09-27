add_executable(filter_lidar main.cpp)
add_dependencies(filter_lidar igvc_msgs_gencpp)
target_link_libraries(filter_lidar ${catkin_LIBRARIES} ${PCL_LIBRARIES})
