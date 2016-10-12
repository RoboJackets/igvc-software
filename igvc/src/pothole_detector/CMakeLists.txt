add_executable(potholedetector main.cpp potholedetector.cpp)
add_dependencies(potholedetector igvc_msgs_gencpp)
target_link_libraries(potholedetector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
