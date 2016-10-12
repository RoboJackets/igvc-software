<<<<<<< HEAD:igvc/src/odometer/CMakeLists.txt
add_executable(odometer main.cpp)
=======
add_executable(odometer main.cpp Odometer.cpp)
add_dependencies(odometer igvc_msgs_gencpp)
>>>>>>> da3eaa7... fixed build dependencies on igvc_msgs package - could certainly be a cleaner solution, but the project should be able to be built first try - closes issue #40:igvc/src/wheel_odometer/CMakeLists.txt
target_link_libraries(odometer ${catkin_LIBRARIES})
