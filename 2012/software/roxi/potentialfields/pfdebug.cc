#include "potentialfields.hpp"
#include "commonVecOps.hpp"
#include <iostream>
using namespace std;
static CvPoint center;
static IplImage* DebugImg = 0;

void potentialfields::pfDebug(double gps_x, double gps_y, double obstacle_x, double obstacle_y)
{
	const double scaleFactor = 20;
	const int w = 400;
	const int h = 400;

	// Rotate vectors to robot frame
	double gps_mag, gps_ang, obs_mag, obs_ang;
	xyToVec(gps_x, gps_y, gps_mag, gps_ang);
	xyToVec(obstacle_x, obstacle_y, obs_mag, obs_ang);
	gps_ang = RotateBearing(gps_ang, -curang);
	obs_ang = RotateBearing(obs_ang, -curang);
	double gps_x_new, gps_y_new, obs_x_new, obs_y_new;
	VecToxy(gps_mag, gps_ang, gps_x_new, gps_y_new);
	VecToxy(obs_mag, obs_ang, obs_x_new, obs_y_new);
	//cout<<"new obs ang: "<<obs_ang<<endl;
	// Flip y's for display
	gps_y_new = - gps_y_new;
	obs_y_new = - obs_y_new;

	// Scale vectors down
	int gps_img_x = (int)(gps_x_new / scaleFactor);
	int gps_img_y = (int)(gps_y_new / scaleFactor);
	int obs_img_x = (int)(obs_x_new / scaleFactor);
	int obs_img_y = (int)(obs_y_new / scaleFactor);

	// Create and move window
	cvNamedWindow("Potential Fields");
	cvMoveWindow( "Potential Fields", 20, 600);


	
	
	
	center = cvPoint(w/2, h/2);
	CvPoint gps_pt = cvPoint(center.x + gps_img_x, center.y + gps_img_y);
	CvPoint obs_pt = cvPoint(center.x + obs_img_x, center.y + obs_img_y);
	cvLine(DebugImg, center, gps_pt, CV_RGB(0,0,255), 5);
	cvLine(DebugImg, center, obs_pt, CV_RGB(255,0,0), 5);

}
void potentialfields::clearDebug()
{

	const int w = 400;
	const int h = 400;
	
	// Create and move window
	cvNamedWindow("Potential Fields");
	cvMoveWindow( "Potential Fields", 20, 600);
	
	// Make image to display
	if(DebugImg){cvReleaseImage(&DebugImg);}
	DebugImg = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3);
	cvSet(DebugImg, CV_RGB(255,255,255));
}

void potentialfields::showDebug()
{

	// Show image and release
	cvShowImage("Potential Fields", DebugImg);
}

void potentialfields::addWorldVec(double gps_x,double gps_y,double scaleFactor,CvScalar color)
{

	const int w = 400;
	const int h = 400;

	// Rotate vectors to robot frame
	double gps_mag, gps_ang;
	xyToVec(gps_x, gps_y, gps_mag, gps_ang);
	
	gps_ang = RotateBearing(gps_ang, -curang);
	
	double gps_x_new, gps_y_new;
	VecToxy(1, gps_ang, gps_x_new, gps_y_new);
	

	// Flip y's for display
	gps_y_new =  gps_y_new;


	// Scale vectors down
	int gps_img_x = (int)(gps_x_new / scaleFactor);
	int gps_img_y = (int)(gps_y_new / scaleFactor);
	
	center = cvPoint(w/2, h/2);
	CvPoint gps_pt = cvPoint(center.x + gps_img_x+2, center.y + gps_img_y);
	cvLine(DebugImg, center, gps_pt, color, 5);
	
}



void potentialfields::addVec(double x,double y)
{
	const double scaleFactor = 10;
	if(DebugImg==0)return;
	
	double m,a;
	
	
	xyToVec(x, y, m, a);
	a = RotateBearing(a, imgAngle);
	a = RotateBearing(a, -curang);
	VecToxy( m, a,x,y);
	y=-y;
	cvLine(DebugImg, center, cvPoint(x * obstacle_weight * meters_per_pixel/scaleFactor+center.x,-y * obstacle_weight * meters_per_pixel/scaleFactor+center.y), CV_RGB(0,255,0), 5);
	// Show image
	cvShowImage("Potential Fields", DebugImg);
}
