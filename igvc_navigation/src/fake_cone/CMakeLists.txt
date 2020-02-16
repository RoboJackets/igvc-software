add_executable(fake_cone fake_cone.cpp main.cpp)
add_dependencies(fake_cone ${catkin_EXPORTED_TARGETS})
target_link_libraries(fake_cone ${catkin_LIBRARIES})

add_library(fakecone_layer
        fakecone_layer.cpp fakecone_layer.h
        fakecone_layer_config.cpp fakecone_layer_config.h
        )
add_dependencies(fakecone_layer ${catkin_EXPORTED_TARGETS})
target_link_libraries(fakecone_layer ${catkin_LIBRARIES})

install(
    TARGETS fake_cone
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
