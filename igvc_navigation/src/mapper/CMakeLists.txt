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

# New interface with costmap2d as a "thin wrapper"
add_library(wrapper_layer wrapper_layer.h wrapper_layer.cpp)
add_dependencies(wrapper_layer ${catkin_EXPORTED_TARGETS})
target_link_libraries(wrapper_layer ${catkin_LIBRARIES} mapper_lib)

find_package(OpenMP)
if (OpenMP_FOUND)
    target_link_libraries(mapper ${OpenMP_CXX_LIBRARIES})
endif ()

install(
    TARGETS mapper
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
