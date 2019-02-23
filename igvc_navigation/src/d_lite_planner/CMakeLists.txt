add_executable(d_lite_planner DLitePlanner.cpp main.cpp)
add_dependencies(d_lite_planner igvc_msgs_gencpp)
target_link_libraries(d_lite_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES} GraphSearch)
