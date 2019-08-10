find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})

add_executable(joystick_driver joystick_driver.cpp)
add_dependencies(joystick_driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick_driver ${catkin_LIBRARIES} ${CURSES_LIBRARIES})

install(
    TARGETS joystick_driver
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
