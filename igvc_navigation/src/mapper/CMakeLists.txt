# Final part of refactor:
add_library(lidar_layer
    lidar_layer.cpp lidar_layer.h
    lidar_layer_config.cpp lidar_layer_config.h
    map_config.cpp map_config.h
    lidar_config.cpp lidar_config.h eigen_hash.h
    gridmap_layer.cpp gridmap_layer.h
    )
add_dependencies(lidar_layer ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_layer ${catkin_LIBRARIES})

add_library(line_layer
    line_layer.cpp line_layer.h
    line_layer_config.cpp line_layer_config.h
    map_config.cpp map_config.h
    camera_config.cpp camera_config.h
    projection_config.cpp projection_config.h
    gridmap_layer.cpp gridmap_layer.h
    )
add_dependencies(line_layer ${catkin_EXPORTED_TARGETS})
target_link_libraries(line_layer ${catkin_LIBRARIES})

add_library(traversability_layer
        traversability_layer.cpp traversability_layer.h
        traversability_layer_config.cpp traversability_layer_config.h
        map_config.cpp map_config.h
        gridmap_layer.cpp gridmap_layer.h
        )

add_library(unrolling_layer
        unrolling_layer.cpp unrolling_layer.h
        unrolling_layer_config.cpp unrolling_layer.h
        )
add_dependencies(unrolling_layer ${catkin_EXPORTED_TARGETS})
target_link_libraries(unrolling_layer ${catkin_LIBRARIES})

install(
    TARGETS lidar_layer line_layer traversability_layer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
