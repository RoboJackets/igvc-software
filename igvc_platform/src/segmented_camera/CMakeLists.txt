find_package(PkgConfig REQUIRED)


add_executable(segmented_camera_info_publisher segmented_camera_info_publisher.cpp)
target_link_libraries(segmented_camera_info_publisher ${catkin_LIBRARIES} ${GLIBTOP_LIBRARIES})
include_directories(${GLIBTOP_INCLUDE_DIRS})
target_compile_options(segmented_camera_info_publisher PUBLIC ${GLIBTOP_CFLAGS_OTHER})
