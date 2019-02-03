add_executable(mapper main.cpp octomapper.cpp octomapper.h)
add_dependencies(mapper igvc_msgs_gencpp)
target_link_libraries(mapper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})
