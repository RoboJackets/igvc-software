find_package(OpenMP)
if (OpenMP_FOUND)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# set the path to the library folder
link_directories(/usr/local/lib)

#Append Gperftools_DIR
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/FindGperftools.cmake)

include_directories(
  ${GPERFTOOLS_INCLUDE_DIR}
)
set (Gperftools_DIR "${CMAKE_CURRENT_LIST_DIR}/cmake/")
find_package(Gperftools REQUIRED)
add_executable(particle_filter
  particle_filter_node.cpp
  octomapper.cpp octomapper.h
  particle_filter.h particle_filter.cpp
  scanmatcher.cpp scanmatcher.h
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/hsluv.h
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/hsluv.c)
add_dependencies(particle_filter igvc_msgs_gencpp)
target_link_libraries(particle_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${GPERFTOOLS_LIBRARIES})
