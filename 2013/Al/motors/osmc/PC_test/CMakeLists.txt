include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

include_directories(
	#for XmlConfiguration
	${IGVC_SOURCE_DIR}/roxi
	
	${IGVC_SOURCE_DIR}/arduino
	${IGVC_SOURCE_DIR}/encoders/pcdriver_signed
	/usr/include/qt4
	/usr/include/qt4/Qt
	/usr/include/qt4/QtGui
	/usr/include/qt4/QtCore
	/usr/include/qt4/QtXml
)

add_library (OSMC_driver OSMC_driver.cc)

target_link_libraries(OSMC_driver 
	ArduinoInterface
	quadCoderDriver_signed
	
	#for XmlConfiguration
	xml
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

add_executable(MagnoTest
	testMagnetometer.cc
)
add_executable(MagOffset
	FindMagOffset.cc
)

target_link_libraries(OSMC_test OSMC_driver)
target_link_libraries(step_test OSMC_driver)
target_link_libraries(dumbctltest OSMC_driver)
target_link_libraries(pdtest OSMC_driver)
target_link_libraries(MagnoTest OSMC_driver)
target_link_libraries(MagOffset OSMC_driver)
