include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

include_directories(${IGVC_SOURCE_DIR}/arduino)

add_library(quadCoderDriver
	quadCoderDriver.cc
)

add_executable(newCoderTest
	test.cc
)

target_link_libraries(quadCoderDriver
	ArduinoInterface
)

target_link_libraries(newCoderTest
	quadCoderDriver
)
