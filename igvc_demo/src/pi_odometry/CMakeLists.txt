find_package(pigpio REQUIRED)
add_executable(pi_odometry pi_odometry.cpp)
target_link_libraries(pi_odometry ${catkin_LIBRARIES} ${pigpio_LIBRARY})
target_include_directories(pi_odometry PRIVATE ${pigpio_INCLUDE_DIRS})
