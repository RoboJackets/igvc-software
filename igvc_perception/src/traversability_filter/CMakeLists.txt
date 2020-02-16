add_executable(traversability_filter traversability_filter.cpp)
add_dependencies(traversability_filter ${catkin_EXPORTED_TARGETS})
target_link_libraries(traversability_filter ${catkin_LIBRARIES})

install(
        TARGETS traversability_filter
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
