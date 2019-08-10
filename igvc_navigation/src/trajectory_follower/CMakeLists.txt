add_executable(trajectory_follower trajectory_follower.cpp)
add_dependencies(trajectory_follower ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_follower ${catkin_LIBRARIES})

install(
    TARGETS trajectory_follower
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
