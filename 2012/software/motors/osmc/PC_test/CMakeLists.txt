include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

include_directories(
	${IGVC_SOURCE_DIR}/arduino
	${IGVC_SOURCE_DIR}/encoders/pcdriver_signed
)

add_library (OSMC_driver OSMC_driver.cc)

target_link_libraries(OSMC_driver 
	ArduinoInterface
	quadCoderDriver_signed
)

add_executable(OSMC_test
	main.cc
)

add_executable(step_test
	stepresp.cc
)

add_executable(dumbctltest
	dumbctl.cc
)

add_executable(pdtest
	pdtest.cc
)

target_link_libraries(OSMC_test OSMC_driver)
target_link_libraries(step_test OSMC_driver)
target_link_libraries(dumbctltest OSMC_driver)
target_link_libraries(pdtest OSMC_driver)
