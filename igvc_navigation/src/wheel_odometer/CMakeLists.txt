add_executable(odometer Odometer.cpp)
add_dependencies(odometer ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometer ${catkin_LIBRARIES})

install(
    TARGETS odometer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
