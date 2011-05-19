include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

include_directories(
	${IGVC_SOURCE_DIR}/arduino
	${IGVC_SOURCE_DIR}/gps
	${IGVC_SOURCE_DIR}/motors/osmc/PC_test
	${IGVC_SOURCE_DIR}/motors/osmc/PC_4wd
	${IGVC_SOURCE_DIR}/lidar
	${IGVC_SOURCE_DIR}/encoders/pcdriver_4wd_signed
	${IGVC_SOURCE_DIR}/encoders/pcdriver_signed
)

add_executable(gps_drive
	main.cc
)

target_link_libraries(gps_drive
	gps
	OSMC_driver_4wd
	NAV200
	lidarProc
)
