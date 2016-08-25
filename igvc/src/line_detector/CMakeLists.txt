add_executable(linedetector main.cpp linedetector.cpp)
target_link_libraries(linedetector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
