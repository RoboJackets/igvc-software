include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

include_directories(${IGVC_SOURCE_DIR}/arduino
${IGVC_SOURCE_DIR}/motors/osmc/PC_test
${IGVC_SOURCE_DIR}/encoders/pcdriver_signed
)

add_library(quadCoderDriver
	quadCoderDriver.cc
)

add_executable(newCoderTest
	test.cc
)

add_executable(getvelocities
	getvelocities.cc
)

target_link_libraries(quadCoderDriver
	ArduinoInterface
)

target_link_libraries(newCoderTest
	quadCoderDriver
)

target_link_libraries(getvelocities
	quadCoderDriver
	OSMC_driver
)




