add_executable(sim_color_detector main.cpp)
add_dependencies(sim_color_detector igvc_msgs_gencpp)
target_link_libraries(sim_color_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
