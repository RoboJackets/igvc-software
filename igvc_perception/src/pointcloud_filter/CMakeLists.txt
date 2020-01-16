add_library(pointcloud_filter_lib STATIC
    pointcloud_filter.cpp
    pointcloud_filter_config.cpp
    bundle.cpp
    back_filter/back_filter_config.cpp
    back_filter/back_filter.cpp
    radius_filter/radius_filter_config.cpp
    radius_filter/radius_filter.cpp
    ground_filter/ground_filter.cpp
    ground_filter/ground_filter_config.cpp
    tf_transform_filter/tf_transform_filter.cpp
    raycast_filter/raycast_filter_config.cpp
    raycast_filter/raycast_filter.cpp
        )
add_dependencies(pointcloud_filter_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_filter_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pointcloud_filter pointcloud_filter_node.cpp)
add_dependencies(pointcloud_filter ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_filter ${catkin_LIBRARIES} pointcloud_filter_lib)

install(
    TARGETS pointcloud_filter pointcloud_filter_lib
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(barrel_seg actual_barrel_segmentation/actual_barrel_segmentation.cpp)
target_link_libraries(barrel_seg ${catkin_LIBRARIES} ${PCL_LIBRARIES})