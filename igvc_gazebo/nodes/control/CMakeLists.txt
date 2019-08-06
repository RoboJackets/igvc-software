add_executable(control main.cpp)
add_dependencies(control ${catkin_EXPORTED_TARGETS})
target_link_libraries(control ${catkin_LIBRARIES})

install(
    TARGETS control
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
