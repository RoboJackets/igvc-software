include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

add_executable(test_kdtree
	test.cc
)

target_link_libraries(test_kdtree
	kdtree
)

add_library(kdtree
	kdtree.c
)
