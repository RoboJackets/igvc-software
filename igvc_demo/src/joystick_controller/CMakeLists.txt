add_executable(joystick_controller joystick_controller.cpp)
add_dependencies(joystick_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick_controller ${catkin_LIBRARIES})

install(
    TARGETS joystick_controller
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
