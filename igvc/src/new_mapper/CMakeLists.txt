add_executable(new_mapper main.cpp)
add_dependencies(new_mapper igvc_msgs_gencpp)
target_link_libraries(new_mapper ${catkin_LIBRARIES} ${PCL_LIBRARIES})
