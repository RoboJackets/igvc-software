#include "potentialfields.hpp"
#include "commonVecOps.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
#include <iostream>

using std::cout;
using std::endl;

void printbitmap(bool* bitmap, int width, int height);

int main()
{
	/*IplImage* img = cvLoadImage("../TestFiles/course6.png");
	// create a window
	cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE); 
	cvMoveWindow("mainWin", 100, 100);
  	// show the image
  	cvShowImage("mainWin", img );
  	// wait for a key
 	//cvWaitKey(0);
  	// release the image
  	//cvReleaseImage(&img);

	/*PFieldNode p1;
	p1.f_score = 100;
	PFieldNode p2;
	p2.f_score = 0;
	bool x = p1 < p2;
	cout << "p1 < p2 : " << x << endl;*/

	/*int height = img->height;
	int width = img->width;
	int step = img->widthStep;
	int channels = img->nChannels;
	uchar* data = (uchar *)img->imageData;
	int index = 0;
	bool bitmap[height*width];
	double thresh = 127.5;
	for(int hindex = 0; hindex < height; hindex++)
	{
		for(int windex = 0; windex < width; windex++)
		{
			int redval = (int)data[hindex*step+windex*channels];
			int greenval = (int)data[hindex*step+windex*channels+1];
			int blueval = (int)data[hindex*step+windex*channels+2];
			double avgval = (redval+greenval+blueval)/3.0;
			if(avgval<thresh)
			{
				bitmap[index] = 1;
			}
			else
			{
				bitmap[index] = 0;
			}
			index++;
		}
	}*/

	/*for(int row=0; row<height; row++)
	{
		for(int col=0; col<width; col++)
		{
			cout << bitmap[row*width+col] << " ";
		}
		cout << endl;
	}*/


	// Testing commonVecOps

	// rad2deg
	/*cout << "rad2deg(pi) = " << rad2deg(M_PI) << endl;
	cout << "rad2deg(pi/4) = " << rad2deg(M_PI/4.0) << endl;
	cout << "rad2deg(3pi/4) = " << rad2deg(3*M_PI/4.0) << endl;
	cout << "rad2deg(3pi/2) = " << rad2deg(3*M_PI/2.0) << endl;
	cout << "rad2deg(7pi/4) = " << rad2deg(7*M_PI/4.0) << endl;
	cout << "rad2deg(4pi/3) = " << rad2deg(4*M_PI/3.0) << endl;*/

	// deg2rad
	/*cout << "deg2rad(180) = " << deg2rad(180) << endl;
	cout << "deg2rad(45) = " << deg2rad(45) << endl;
	cout << "deg2rad(135) = " << deg2rad(135) << endl;
	cout << "deg2rad(270) = " << deg2rad(270) << endl;
	cout << "deg2rad(315) = " << deg2rad(315) << endl;
	cout << "deg2rad(240) = " << deg2rad(240) << endl;*/

	// vec2bear
	/*cout << "vec2bear(0) = " << vec2bear(0) << endl;
	cout << "vec2bear(45) = " << vec2bear(45) << endl;
	cout << "vec2bear(90) = " << vec2bear(90) << endl;
	cout << "vec2bear(135) = " << vec2bear(135) << endl;
	cout << "vec2bear(180) = " << vec2bear(180) << endl;
	cout << "vec2bear(225) = " << vec2bear(225) << endl;
	cout << "vec2bear(270) = " << vec2bear(270) << endl;
	cout << "vec2bear(315) = " << vec2bear(315) << endl;
	cout << "vec2bear(360) = " << vec2bear(360) << endl;
	cout << "vec2bear(-45) = " << vec2bear(-45) << endl;
	cout << "vec2bear(-90) = " << vec2bear(-90) << endl;
	cout << "vec2bear(-135) = " << vec2bear(-135) << endl;
	cout << "vec2bear(-180) = " << vec2bear(-180) << endl;
	cout << "vec2bear(-225) = " << vec2bear(-225) << endl;
	cout << "vec2bear(-270) = " << vec2bear(-270) << endl;
	cout << "vec2bear(-315) = " << vec2bear(-315) << endl;
	cout << "vec2bear(-360) = " << vec2bear(-360) << endl;*/

	// xyToVec
	/*double mag, ang;
	xyToVec(1, 1, mag, ang);
	cout << "xyToVec(1,1)" << "\nmag: " << mag << "\nang: " << ang << endl;
	xyToVec(1, -1, mag, ang);
	cout << "xyToVec(1,-1)" << "\nmag: " << mag << "\nang: " << ang << endl;
	xyToVec(-1, -1, mag, ang);
	cout << "xyToVec(-1,-1)" << "\nmag: " << mag << "\nang: " << ang << endl;
	xyToVec(-1, 1, mag, ang);
	cout << "xyToVec(-1,1)" << "\nmag: " << mag << "\nang: " << ang << endl;*/

	// VecToxy
	/*double x, y;
	VecToxy(sqrt(2), 45, x, y);
	cout << "VecToxy(sqrt(2),45)" << "\nx: " << x << "\ny: " << y << endl;
	VecToxy(sqrt(2), 135, x, y);
	cout << "VecToxy(sqrt(2),135)" << "\nx: " << x << "\ny: " << y << endl;
	VecToxy(sqrt(2), 225, x, y);
	cout << "VecToxy(sqrt(2),225)" << "\nx: " << x << "\ny: " << y << endl;
	VecToxy(sqrt(2), 315, x, y);
	cout << "VecToxy(sqrt(2),315)" << "\nx: " << x << "\ny: " << y << endl;*/

	// AddVecs
	/*double xvals[4] = {2, 3.5, -.5, 0};
	double yvals[4] = {1, -6, 7.5, 2};
	double xnet, ynet;
	AddVecs(xvals, yvals, 4, xnet, ynet);
	cout << "AddVecs((2,1),(3.5,-6),(-.5,7.5),(0,2))" << "\nxnet: " << xnet << "\nynet: " << ynet << endl;*/

	// Distance2D
	/*cout << "Distance2D(0,0,-4,3) = " << Distance2D(0,0,-4,3) << endl;
	cout << "Distance2D(-5, 2, -3, 2-sqrt(12)) = " << Distance2D(-5, 2, -3, 2-sqrt(12)) << endl;
	cout << "Distance2D(5, 5, -5, -5) = " << Distance2D(5, 5, -5, -5) << endl;	*/

	// RotateBearing
	/*cout << "RotateBearing(45,-90) = " << RotateBearing(45, -90) << endl;
	cout << "RotateBearing(350,44) = " << RotateBearing(350, 44) << endl;
	cout << "RotateBearing(120,55) = " << RotateBearing(120, 55) << endl;
	cout << "RotateBearing(300,-20) = " << RotateBearing(300, -20) << endl;*/

	// Will only run if TESTINGMODE is defined, meaning that the functions of class potentialfields have been made
	// public for testing purposes. 

	double ang1 = 0, ang2 = -32, ang3 = 179, ang4 = 181, ang5 = 270;
	ang1 = clampVector(ang1, 30);
	ang2 = clampVector(ang2, 30);
	ang3 = clampVector(ang3, 30);
	ang4 = clampVector(ang4, 30);
	ang5 = clampVector(ang5, 30);
	cout << "ang1 " << ang1 << " ang2 " << ang2 << " ang3 " << ang3 << " ang4 " << ang4 << " ang5 " << ang5 << endl;

	// Constructor
	potentialfields pf;
	// Test getObstacleVec
	bool newbitmap[9] = {0, 1, 0, 0, 1, 0, 0, 0, 0};
	int x0 = 1, y0 = 2;
	int radius = 10;
	double mag, ang, xvel, yvel;
	ReturnData newdata;;
	pf.imgAngle = 90;//90;
	pf.curang = 180;//180;
	pf.xsize = 3;
	pf.ysize = 3;
	pf.getObstacleVector(x0, y0, radius, newbitmap, &newdata);
	xyToVec(newdata.x_vel, newdata.y_vel, mag, ang);//in map coordinates
	ang = RotateBearing(ang, pf.imgAngle);
	VecToxy(mag,ang, xvel, yvel);
	cout << "x_vel " << xvel << "\ny_vel " << yvel << endl;
	//pf.xsize = width;
	//pf.ysize = height;
	/*int numgoals = pf.GPS_Goals.size();
	for(int i=0; i<numgoals; i++)
	{
		cout << "GPS goal " << i+1 << endl;
		cout << "\tlat: " << pf.GPS_Goals[i].lat << endl;
		cout << "\tlon: " << pf.GPS_Goals[i].lon << endl;
		cout << "\tang: " << pf.GPS_Goals[i].ang << endl << endl;
	}*/

	// dropWaypoint
	/*pf.dropWaypoint(42.6777,-83.1951,90);
	pf.dropWaypoint(42.6790,-83.1942,45);
	pf.dropWaypoint(42.69994,-83.1940,0);
	CvPoint robotBaseAt = CvPoint();
	robotBaseAt.x = 320;
	robotBaseAt.y = 340;

	CvPoint robotLookingAt = CvPoint();
	robotLookingAt.x = 320;
	robotLookingAt.y = 1;

	Point2D<int> goal = Point2D<int>();
	
	//pf.getVectorMotor(img, 0, robotBaseAt, robotLookingAt, goal);
	pf.getCompleteVector(img, 0, robotBaseAt, robotLookingAt, goal);

	cout << "goal.x = " << goal.x << endl;
	cout << "goal.y = " << goal.y << endl;

	//numgoals = pf.GPS_Goals.size();
	/*for(int i=0; i<numgoals; i++)
	{
		cout << "GPS goal " << i+1 << endl;
		cout << "\tlat: " << pf.GPS_Goals[i].lat << endl;
		cout << "\tlon: " << pf.GPS_Goals[i].lon << endl;
		cout << "\tang: " << pf.GPS_Goals[i].ang << endl << endl;
	}
	int numprevs = pf.GPS_Prev_Loc.size();
	for(int i=0; i<numprevs; i++)
	{
		cout << "GPS Location " << i+1 << endl;
		cout << "\tlat: " << pf.GPS_Prev_Loc[i].lat << endl;
		cout << "\tlon: " << pf.GPS_Prev_Loc[i].lon << endl;
		cout << "\tang: " << pf.GPS_Prev_Loc[i].ang << endl << endl;
	}*/

	#ifdef TESTINGMODE
	// removeclumps
	// void removeclumps(bool* obstacles);
	// TODO: Test once function is written

	// radiusfix
	/*printbitmap(bitmap, width, height);
	cout << endl;
	pf.radiusfix(bitmap);
	printbitmap(bitmap, width, height);
	cout << endl;*/

	// get2Dindexvalue
	/*printbitmap(bitmap, width, height);	
	cout << endl;
	for(int y=0; y<height; y++)
	{
		for(int x=0; x<width; x++)
		{
			cout << pf.get2Dindexvalue(bitmap,x,y) << " ";
		}
		cout << endl;
	}
	cout << endl;*/

	// set2Dindexvalue
	/*printbitmap(bitmap, width, height);
	cout << endl;
	for(int y=0; y<height; y++)
	{
		for(int x=0; x<width; x++)
		{
			pf.set2Dindexvalue(bitmap,x,y,1);
		}
	}	
	printbitmap(bitmap, width, height);
	cout << endl;*/

	// fillinRadius
	/*pf.fillinRadius(bitmap, 49, 49, 10);
	printbitmap(bitmap, width, height);
	cout << endl;*/

	// convertToRealxy
	/*double xcomp = 2;
	double ycomp = 2;	
	pf.convertToRealxy(xcomp, ycomp);
	cout << "convertToRealxy(2,2)" << endl << "xcomp: " << xcomp << endl << "ycomp: " << ycomp << endl;
	xcomp = -4.6;
	ycomp = 0;
	pf.convertToRealxy(xcomp, ycomp);
	cout << "convertToRealxy(-4.6,0)" << endl << "xcomp: " << xcomp << endl << "ycomp: " << ycomp << endl;*/

	// convertToRealVec
	/*double vel = 5.6;
	double ang = 89.5;
	pf.convertToRealVec(vel, ang);
	cout << "convertToRealxy(5.6,89.5)" << endl << "vel: " << vel << endl << "ang: " << ang << endl;
	vel = 23.5;
	ang = 134.6;
	pf.convertToRealVec(vel, ang);
	cout << "convertToRealVec(23.5,134.6)" << endl << "vel: " << vel << endl << "ang: " << ang << endl;*/

	// updateCurLocation
	/*pf.dropWaypoint(42.6777,-83.1951,90);
	pf.updateCurLocation();
	cout << "curlat: " << pf.curlat << endl << "curlon: " << pf.curlon << endl << "imgAngle: " << pf.imgAngle << endl;*/

	// distBtwGPSPoints
	/*GPS_point pa(50+3/60+59/3600,5+42/60+53/3600);
	GPS_point pb(58+38/60+38/3600,3+4/60+12/3600);
	cout << "distBtwGPSPpoints(point1, point2)\nlat1: 50 03 59\nlon1: 005 42 53\nlat2: 58 38 38\n lon2: 003 04 12\n";
	cout << "distance = " << pf.distBtwGPSPoints(pa, pb) << endl;*/

	// attractorPixels
	/*double x_vel, y_vel;
	pf.attractorPixels(0, 0, 10, 10, x_vel, y_vel);
	cout << "attractorPixels(0,0,10,10): " << endl;
	cout << "x_vel: " << x_vel << endl << "y_vel: " << y_vel << endl;
	pf.attractorPixels(5, 5, 0, 0, x_vel, y_vel);
	cout << "attractorPixels(5,5,0,0): " << endl;
	cout << "x_vel: " << x_vel << endl << "y_vel: " << y_vel << endl;*/

	// repulsivePixels
	/*cout << "repulsivePixels(0,0,10,10,20): " << endl;
	pf.repulsivePixels(0, 0, 10, 10, 20, x_vel, y_vel);
	cout << "x_vel: " << x_vel << endl << "y_vel: " << y_vel << endl;
	cout << "repulsivePixels(5,5,0,0,20): " << endl;
	pf.repulsivePixels(5, 5, 0, 0, 20, x_vel, y_vel);
	cout << "x_vel: " << x_vel << endl << "y_vel: " << y_vel << endl;*/

	// doSomethingforIndexesInRadius
	/* Don't really need to test this, this is used in other places */

	// mediaThreshFileter
	/* TODO: test this
	void medianThreshFileter(bool* array, int thresh_size); */

	// getAvoidVec
	//printbitmap(bitmap, width, height);
	/*pf.robotlocx = 320;
	pf.robotlocy = 240;
	double xvel, yvel;
	pf.getAvoidVec(bitmap, xvel, yvel);
	cout << "getAvoidVec(bitmap): " << endl << "xvel: " << xvel << endl << "yvel: " << yvel << endl;
	cout << "x/y: " << xvel/yvel << endl;*/

	// getImgTargTargetVec
	/*pf.getImgTargetVec(bitmap, xvel, yvel);
	cout << "getImgTargetVec(bitmap): " << endl << "xvel: " << xvel << endl << "yvel: " << yvel << endl;
	cout << "x/y: " << xvel/yvel << endl;*/

	// Can't do these until we have function for angles between GPS points
	// TODO: Once these functions are done, test these
	/*void getGPSTargetVec(double& xvel, double& yvel);
	void getGPSAvoidVec(double& xvel, double& yvel);*/	

	#endif
	
	return 0;
}

void printbitmap(bool* bitmap, int width, int height)
{
	for(int row=0; row<height; row++)
	{
		for(int col=0; col<width; col++)
		{
			cout << bitmap[row*width+col] << " ";
		}
		cout << endl;
	}
}
