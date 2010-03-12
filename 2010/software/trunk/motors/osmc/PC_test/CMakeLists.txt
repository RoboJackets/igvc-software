include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

include_directories(${IGVC_SOURCE_DIR}/arduino)
include_directories(${IGVC_SOURCE_DIR}/newencoders/pcdriver)

add_library (OSMC_driver OSMC_driver.cc)

target_link_libraries(OSMC_driver ArduinoInterface)

add_executable(OSMC_test
	main.cc
)

add_executable(step_test
	stepresp.cc
)

add_executable(dumbctltest
	dumbctl.cc
)

target_link_libraries(OSMC_test OSMC_driver)
target_link_libraries(step_test OSMC_driver quadCoderDriver)
target_link_libraries(dumbctltest OSMC_driver quadCoderDriver)
