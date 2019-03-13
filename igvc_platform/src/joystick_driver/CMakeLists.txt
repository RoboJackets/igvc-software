find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})

add_executable(joystick_driver joystick_driver.cpp joystick_driver.h)
add_dependencies(joystick_driver igvc_msgs_gencpp)
target_link_libraries(joystick_driver ${catkin_LIBRARIES} ${CURSES_LIBRARIES})
