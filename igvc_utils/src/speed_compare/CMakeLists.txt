add_executable(speed_compare main.cpp)
add_dependencies(speed_compare ${catkin_EXPORTED_TARGETS})
target_link_libraries(speed_compare ${catkin_LIBRARIES})

install(
    TARGETS speed_compare
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
