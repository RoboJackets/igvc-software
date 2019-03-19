add_executable(motor_controller
               ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/nanopb/pb_decode.c
               ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/nanopb/pb_common.c
               ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/nanopb/pb_encode.c
               ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/nanopb/protos/igvc.pb.c
               motor_controller.cpp)

add_dependencies(motor_controller igvc_msgs_gencpp)

target_link_libraries(motor_controller ${catkin_LIBRARIES} SerialPort)
target_link_libraries(motor_controller ${catkin_LIBRARIES} EthernetSocket)
