add_executable(pidtester main.cpp)
add_dependencies(pidtester igvc_msgs_gencpp)
target_link_libraries(pidtester ${catkin_LIBRARIES})

add_executable(encrecord encrecord.cpp)
add_dependencies(encrecord igvc_msgs_gencpp)
target_link_libraries(encrecord ${catkin_LIBRARIES})
