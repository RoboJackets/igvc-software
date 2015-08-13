add_executable(linedetector main.cpp linedetector.cpp)
target_link_libraries(linedetector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test_line_detector test_line_detector.cpp)
target_link_libraries(test_line_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
