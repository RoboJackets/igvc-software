add_executable(pidtester main.cpp)
target_link_libraries(pidtester ${catkin_LIBRARIES})

add_executable(encrecord encrecord.cpp)
target_link_libraries(encrecord ${catkin_LIBRARIES})
