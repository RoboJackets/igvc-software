add_executable(local_mapper main.cpp)
add_dependencies(local_mapper igvc_msgs_gencpp)
target_link_libraries(local_mapper ${catkin_LIBRARIES} ${PCL_LIBRARIES})
