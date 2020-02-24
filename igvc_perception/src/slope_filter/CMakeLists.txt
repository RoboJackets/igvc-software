add_executable(slope_filter slope_filter.cpp)
add_dependencies(slope_filter ${catkin_EXPORTED_TARGETS})
target_link_libraries(slope_filter ${catkin_LIBRARIES})

install(
        TARGETS slope_filter
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
