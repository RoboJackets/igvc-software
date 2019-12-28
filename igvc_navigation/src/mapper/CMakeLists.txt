# Mapper Library
add_library(mapper_lib
    ros_mapper.cpp
    octomapper.cpp
    octomapper.h
    ros_mapper.h
    map_utils.cpp
    map_utils.h
    mapper.cpp
    mapper.h)
add_dependencies(mapper_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapper_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

# Old interface, where mapper is an independent node
add_executable(mapper mapper_node.cpp)
add_dependencies(mapper ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} mapper_lib)

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
        fakecone.cpp fakecone.h fakecone_layer.cpp fakecone_layer.h fakecone_layer_config.cpp fakecone_layer_config.h)
add_dependencies(line_layer ${catkin_EXPORTED_TARGETS})
target_link_libraries(line_layer ${catkin_LIBRARIES})

add_library(unrolling_layer
        unrolling_layer.cpp unrolling_layer.h
        unrolling_layer_config.cpp unrolling_layer.h
        )
add_dependencies(unrolling_layer ${catkin_EXPORTED_TARGETS})
target_link_libraries(unrolling_layer ${catkin_LIBRARIES})

find_package(OpenMP)
if (OpenMP_FOUND)
    target_link_libraries(mapper ${OpenMP_CXX_LIBRARIES})
endif ()

install(
    TARGETS mapper lidar_layer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
