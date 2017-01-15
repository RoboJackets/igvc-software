add_executable(scan_to_image main.cpp)
target_link_libraries(scan_to_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
