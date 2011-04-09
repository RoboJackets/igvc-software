include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

include_directories(${IGVC_SOURCE_DIR}/arduino)

add_library(NAV200
	NAV200.cpp
)

target_link_libraries(NAV200
	usb
)

add_library(lidarProc
	lidarProc.cc
)

target_link_libraries(lidarProc
	cv
)

add_executable(test
	test.cc
)

target_link_libraries(test
	NAV200
	lidarProc
)

add_executable(lintest
	find_linear_test.cc
)

target_link_libraries(lintest
	NAV200
)

add_executable(datadump
	datadump.cc
)

target_link_libraries(datadump
	NAV200
)

add_executable(lidartestgui
	lidartestgui.cc
)

target_link_libraries(lidartestgui
	NAV200
	lidarProc
	cv
	highgui

	#mgl-fltk
	mgl-glut

	boost_thread-mt
)
