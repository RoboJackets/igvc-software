
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
	opencv_core
	opencv_imgproc
	opencv_highgui
	opencv_calib3d
	
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
	opencv_core
	opencv_imgproc
	opencv_highgui
)

#webcam demo
add_executable(wcdemo
cvwebcamdemo.cpp
)
target_link_libraries(wcdemo
	opencv_core
	opencv_imgproc
	opencv_highgui
)

#perspective transform demo
add_executable(ptdemo
persptranscvdemo.cpp
)

target_link_libraries(ptdemo
	opencv_core
	opencv_imgproc
	opencv_highgui
)

#glut perspective transform demo
add_executable(gptdemo
gldemo.cpp
)

target_link_libraries(gptdemo
	opencv_core
	opencv_imgproc
	opencv_highgui
	glut
)
#glut perspective transform demo with firewire
add_executable(fwgldemo
fwgldemo.cpp
)
target_link_libraries(fwgldemo
	opencv_core
	opencv_imgproc
	opencv_highgui
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
	opencv_core
	opencv_imgproc
	opencv_highgui
	glut
	
	
	roxi_vision
	QtCore
	QtXml
	QtGui
	xml
)
