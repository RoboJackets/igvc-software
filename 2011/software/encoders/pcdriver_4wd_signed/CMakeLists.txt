include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

include_directories(
	${IGVC_SOURCE_DIR}/arduino
	${IGVC_SOURCE_DIR}/newencoders/pcdriver_signed
)

add_library(quadCoderDriver_4wd_signed
	quadCoderDriver_4wd_signed.cc
)

add_executable(newCoderTest_4wd_signed
	test.cc
)

target_link_libraries(quadCoderDriver_4wd_signed
	quadCoderDriver_signed
)

target_link_libraries(newCoderTest_4wd_signed
	quadCoderDriver_4wd_signed
)
