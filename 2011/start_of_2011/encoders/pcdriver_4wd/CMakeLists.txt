include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

include_directories(${IGVC_SOURCE_DIR}/arduino)

add_library(quadCoderDriver_4wd
	quadCoderDriver.cc
)

add_executable(newCoderTest_4wd
	test.cc
)

target_link_libraries(quadCoderDriver_4wd
	ArduinoInterface
)

target_link_libraries(newCoderTest_4wd
	quadCoderDriver_4wd
)
