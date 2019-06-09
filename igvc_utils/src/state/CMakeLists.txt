add_library(igvc_state robot_control.cpp robot_state.cpp)
add_dependencies(igvc_state igvc_msgs_gencpp)
target_link_libraries(igvc_state ${catkin_LIBRARIES})
