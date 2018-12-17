add_executable(field_d_pathplanner main.cpp Node.cpp Cell.cpp)
add_dependencies(field_d_pathplanner igvc_msgs_gencpp)
target_link_libraries(field_d_pathplanner ${catkin_LIBRARIES} ${PCL_LIBRARIES})
