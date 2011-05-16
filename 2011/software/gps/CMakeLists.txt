include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

add_library(gps_test
	gps_test.cc
)

add_library(gps
	gps.cc
)

target_link_libraries(gps
	boost_asio
)
