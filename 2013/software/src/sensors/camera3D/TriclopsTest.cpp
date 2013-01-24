//============================================================================
// Name        : TriclopsTest.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>

#include <triclops.h>

#include <flycapture/FlyCapture2.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace FlyCapture2;

void handleError(TriclopsError error, string msg) {
	if(error != TriclopsErrorOk) {
		std::cout << "Error: " << msg << std::endl;
		exit( 1 );
	}
}

void handleError(TriclopsBool error, string msg) {
	if(!error) {
		std::cout << "Error: " << msg << std::endl;
		exit( 1 );
	}
}

int main() {

	TriclopsContext context;
	TriclopsImage depthImage;
	TriclopsInput inputData;

	BusManager busMgr;
	unsigned int numCameras;
	PGRGuid guid;
	if(busMgr.GetNumOfCameras(&numCameras) == PGRERROR_OK) {
		if(numCameras > 0) {

			if(busMgr.GetCameraFromIndex(0, &guid) != PGRERROR_OK) {
				std::cout << "Error getting camera.";
				exit(1);
			}

		} else {
			std::cout << "No camera detected." << std::endl;
			exit(1);
		}
	} else {
		std::cout << "Error detecting camera." << std::endl;
		exit(1);
	}

	return 0;
}
