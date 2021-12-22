add_executable(motor_controller
                ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/nanopb/pb_decode.c
                ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/nanopb/pb_common.c
                ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/nanopb/pb_encode.c
#                ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/nanopb/protos/igvc.pb.c
                ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/nanopb/protos/new_swerve_commands.pb.c

                motor_controller.cpp)

add_dependencies(motor_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(motor_controller ${catkin_LIBRARIES})

install(
    TARGETS motor_controller
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
