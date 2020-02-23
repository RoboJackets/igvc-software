add_library(fakecone_layer
        fakecone_layer.cpp fakecone_layer.h
        fakecone_layer_config.cpp fakecone_layer_config.h
        fake_cone.h fake_cone.cpp
        )
add_dependencies(fakecone_layer ${catkin_EXPORTED_TARGETS})
target_link_libraries(fakecone_layer ${catkin_LIBRARIES})


install(
    TARGETS fakecone_layer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
