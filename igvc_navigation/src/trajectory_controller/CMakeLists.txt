add_library(TestLib ${LIB_TYPE} helloWorld.cu)

add_executable(trajectory_controller main.cpp trajectory_controller.cpp ros_trajectory_controller.cpp ros_trajectory_controller.h trajectory_utils.cpp trajectory_utils.h)
add_dependencies(trajectory_controller igvc_msgs_gencpp TestLib)
target_link_libraries(trajectory_controller ${catkin_LIBRARIES} TestLib)
