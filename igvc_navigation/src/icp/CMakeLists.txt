add_executable(icp icp.cpp icp.h)
add_dependencies(icp igvc_msgs_gencpp)
target_link_libraries(icp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
