add_library(back_up_recovery back_up_recovery.cpp)
add_dependencies(back_up_recovery ${catkin_EXPORTED_TARGETS})
target_link_libraries(back_up_recovery ${catkin_LIBRARIES})

install(
        TARGETS back_up_recovery
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
