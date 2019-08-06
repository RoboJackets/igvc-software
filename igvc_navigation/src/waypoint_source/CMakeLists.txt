add_executable(waypoint_source waypoint_source.cpp)
add_dependencies(waypoint_source ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_source ${catkin_LIBRARIES})

install(
    TARGETS waypoint_source
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
