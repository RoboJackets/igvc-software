add_executable(speed_compare main.cpp)
add_dependencies(speed_compare igvc_msgs_gencpp)
target_link_libraries(speed_compare ${catkin_LIBRARIES})

add_executable(idk idk.cpp)
add_dependencies(idk igvc_msgs_gencpp)
target_link_libraries(idk ${catkin_LIBRARIES})
