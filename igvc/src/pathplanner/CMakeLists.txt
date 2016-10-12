add_executable(pathplanner main.cpp igvcsearchproblem.cpp searchlocation.cpp searchmove.cpp)
add_dependencies(pathplanner igvc_msgs_gencpp)
target_link_libraries(pathplanner ${catkin_LIBRARIES} ${PCL_LIBRARIES})
