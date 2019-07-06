add_library(igvc_state robot_control.cpp robot_state.cpp)
add_dependencies(igvc_state ${catkin_EXPORTED_TARGETS})
target_link_libraries(igvc_state ${catkin_LIBRARIES})

install(
    TARGETS igvc_state
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
