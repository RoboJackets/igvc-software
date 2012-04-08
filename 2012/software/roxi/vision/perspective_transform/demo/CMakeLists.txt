include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)
include_directories(
	${IGVC_SOURCE_DIR}/roxi/vision/perspective_transform
	/usr/include/opencv
)

#checker calibrater
add_executable(ckcalib
checkercalib.cpp
)
target_link_libraries(ckcalib
	cv
	cvaux
	highgui
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
