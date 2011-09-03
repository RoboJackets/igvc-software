include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

include_directories(${IGVC_SOURCE_DIR}/arduino)
include_directories(${IGVC_SOURCE_DIR}/encoders/pcdriver_signed)
include_directories(${IGVC_SOURCE_DIR}/motors/osmc/PC_test)

add_library (OSMC_driver_4wd OSMC_4wd_driver.cc)

target_link_libraries(OSMC_driver_4wd OSMC_driver)

add_executable(OSMC_test_4wd
	main.cc
)

add_executable(OSMC_test_4wd_2
	test2.cc
)

add_executable(step_test_4wd
	stepresp.cc
)

add_executable(light_test_4wd
	lighttest.cc
)

#add_executable(dumbctltest_4wd
#	dumbctl.cc
#)

target_link_libraries(OSMC_test_4wd OSMC_driver_4wd)
target_link_libraries(OSMC_test_4wd_2 OSMC_driver_4wd)
target_link_libraries(step_test_4wd OSMC_driver_4wd)
target_link_libraries(light_test_4wd OSMC_driver_4wd)
#target_link_libraries(dumbctltest_4wd OSMC_driver_4wd)
