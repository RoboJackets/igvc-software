add_executable(path_follower path_follower.cpp smooth_control.cpp path_follower.h)
add_dependencies(path_follower ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_follower ${catkin_LIBRARIES})

install(
    TARGETS path_follower
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
