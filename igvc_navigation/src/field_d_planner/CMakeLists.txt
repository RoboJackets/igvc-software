add_executable(incremental_planner Node.cpp Graph.cpp PriorityQueue.cpp DLitePlanner.cpp main.cpp)
add_dependencies(incremental_planner igvc_msgs_gencpp)
target_link_libraries(incremental_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES})
