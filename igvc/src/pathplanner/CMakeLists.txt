add_executable(pathplanner main.cpp igvcsearchproblem.cpp searchlocation.cpp searchmove.cpp)
target_link_libraries(pathplanner ${catkin_LIBRARIES} ${PCL_LIBRARIES})
