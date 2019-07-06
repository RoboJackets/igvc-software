add_executable(trajectory_planner trajectory_planner.cpp smooth_control.cpp motion_profiler.cpp motion_profiler.h)
add_dependencies(trajectory_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_planner ${catkin_LIBRARIES})

install(
    TARGETS trajectory_planner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
