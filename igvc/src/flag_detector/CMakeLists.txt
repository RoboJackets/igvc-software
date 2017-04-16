add_executable(flagdetector main.cpp flagdetector.cpp)
target_link_libraries(flagdetector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
