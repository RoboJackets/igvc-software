include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)


add_library(robot_position
	RobotPosition.cpp
	MagnetometerTracking.cpp
	EncoderTracking.cpp	
)

add_library(Coordinates
	Coordinates.cpp
)

target_link_libraries(Coordinates
	#CV stuff
	cv
	cvaux
	highgui

)


target_link_libraries(robot_position
	boost_system
	gps
	OSMC_driver
	boost_thread
	IMU_Control
)



