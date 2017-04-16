add_executable(global_mapper main.cpp)
add_dependencies(global_mapper igvc_msgs_gencpp)
target_link_libraries(global_mapper ${catkin_LIBRARIES} ${PCL_LIBRARIES})
