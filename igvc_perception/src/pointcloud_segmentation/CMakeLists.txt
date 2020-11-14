add_executable(ground_filter ground_filter.cpp)
add_dependencies(ground_filter ${catkin_EXPORTED_TARGETS})
target_link_libraries(ground_filter ${catkin_LIBRARIES})

install(
        TARGETS ground_filter
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
