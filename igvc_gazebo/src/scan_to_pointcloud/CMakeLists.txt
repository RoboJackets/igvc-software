add_executable(scan_to_pointcloud main.cpp)
add_dependencies(scan_to_pointcloud igvc_msgs_gencpp)
target_link_libraries(scan_to_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})
