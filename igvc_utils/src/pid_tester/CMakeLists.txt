add_executable(pidtester main.cpp)
add_dependencies(pidtester ${catkin_EXPORTED_TARGETS})
target_link_libraries(pidtester ${catkin_LIBRARIES})

add_executable(encrecord encrecord.cpp)
add_dependencies(encrecord ${catkin_EXPORTED_TARGETS})
target_link_libraries(encrecord ${catkin_LIBRARIES})


install(
    TARGETS pidtester
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS encrecord
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
