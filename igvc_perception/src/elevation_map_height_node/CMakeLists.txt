add_executable(elevation_map_height_node elevation_map_height_node.cpp)
add_dependencies(elevation_map_height_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(elevation_map_height_node ${catkin_LIBRARIES})

install(
        TARGETS elevation_map_height_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
