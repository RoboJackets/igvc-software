add_executable(field_d_planner FieldDPlanner.cpp main.cpp)
add_dependencies(field_d_planner igvc_msgs_gencpp)
target_link_libraries(field_d_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES} GraphSearch)
