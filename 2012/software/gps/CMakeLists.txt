include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

add_executable(gps_test
	gps_test.cc
)

target_link_libraries(gps_test
	gps
)

add_executable(gyro_test
	gyro_test.cc
)

target_link_libraries(gyro_test
	gyro
)

add_executable(lambert_test
	lambert_test.cc
)

target_link_libraries(lambert_test
	gps
)

add_library(gps
	gps.cc
	gps_common.cc
)

target_link_libraries(gps
	nmea
	boost_thread
	boost_system
)

add_library(gyro
	gyro.cc
)

target_link_libraries(gyro
	nmea
	boost_thread
	boost_system
)

add_library(nmea
	nmea.cc
)

add_library(IMU_Control
IMU_Control.cc
)
