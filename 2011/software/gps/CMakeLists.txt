include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

add_executable(gps_test
	gps_test.cc
)

target_link_libraries(gps_test
	gps
)

add_library(gps
	gps.cc
)

target_link_libraries(gps
	nmea
	boost_thread
	boost_system
)

add_library(nmea
	nmea.cc
)
