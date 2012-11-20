include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

include_directories(${IGVC_SOURCE_DIR}/arduino)

add_library(quadCoderDriver_signed
	quadCoderDriver_signed.cc
)

add_executable(newCoderTest_signed
	test.cc
)

target_link_libraries(quadCoderDriver_signed
	ArduinoInterface
)

target_link_libraries(newCoderTest_signed
	quadCoderDriver_signed
)
