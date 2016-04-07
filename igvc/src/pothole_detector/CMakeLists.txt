add_executable(potholedetector main.cpp potholedetector.cpp)
target_link_libraries(potholedetector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
