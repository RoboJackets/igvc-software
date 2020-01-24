add_executable(navigation_server navigation_server.cpp)
add_dependencies(navigation_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigation_server ${catkin_LIBRARIES})

install(
    TARGETS navigation_server
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
