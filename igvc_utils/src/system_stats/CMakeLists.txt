add_executable(system_stats main.cpp cpu_usage.hpp)
add_dependencies(system_stats ${catkin_EXPORTED_TARGETS})
target_link_libraries(system_stats ${catkin_LIBRARIES})

install(
    TARGETS system_stats
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
