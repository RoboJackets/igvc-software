add_executable(linedetector main.cpp linedetector.cpp)
add_dependencies(linedetector igvc_msgs_gencpp)
target_link_libraries(linedetector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
