include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

include_directories(${IGVC_SOURCE_DIR}/arduino)

add_library(NAV200
	NAV200.cpp
)

target_link_libraries(NAV200
	usb
)

add_executable(test
	test.cc
)

target_link_libraries(test
	NAV200
)

