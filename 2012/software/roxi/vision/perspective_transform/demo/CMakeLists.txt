
include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)
include_directories(
	${IGVC_SOURCE_DIR}/roxi/vision/perspective_transform
	/usr/include/opencv
	${IGVC_SOURCE_DIR}/roxi/vision
)

#checker calibrater
add_executable(ckcalib
checkercalib.cpp
)
target_link_libraries(ckcalib
	cv
	cvaux
	highgui
	
	roxi_vision
	QtCore
	QtXml
	QtGui
	xml
)

#line draw demo
add_executable(ldemo
linedemo.cpp
)
target_link_libraries(ldemo
	cv
	cvaux
	highgui
)

#webcam demo
add_executable(wcdemo
cvwebcamdemo.cpp
)
target_link_libraries(wcdemo
	cv
	cvaux
	highgui
)

#perspective transform demo
add_executable(ptdemo
persptranscvdemo.cpp
)

target_link_libraries(ptdemo
	cv
	cvaux
	highgui
)

#glut perspective transform demo
add_executable(gptdemo
gldemo.cpp
)

target_link_libraries(gptdemo
	cv
	cvaux
	highgui
	glut
)
#glut perspective transform demo with firewire
add_executable(fwgldemo
fwgldemo.cpp
)
target_link_libraries(fwgldemo
	cv
	cvaux
	highgui
	glut
	
	
	roxi_vision
	QtCore
	QtXml
	QtGui
	xml
)

#glut perspective transform demo with firewire that loads perspective transform from file
add_executable(loaddemo
loaddemo.cpp
)
target_link_libraries(loaddemo
	cv
	cvaux
	highgui
	glut
	
	
	roxi_vision
	QtCore
	QtXml
	QtGui
	xml
)
