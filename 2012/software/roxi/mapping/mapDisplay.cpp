#include "mapDisplay.hpp"

mapDisplay::mapDisplay(GPSState reference, CvImage* startImage)
{	
/*
	XmlConfiguration cfg("Config.xml");
	meters_per_pixel=cfg.getFloat("meters_per_pixel");
*/
	lastImage=(IplImage*)startImage;
	referencePoint=reference;
	cvNamedWindow("Field Map",CV_WINDOW_AUTOSIZE);
	cvMoveWindow("Field Map",X_COORD_ON_SCREEN ,Y_COORD_ON_SCREEN);
	cvResizeWindow("Field Map",WIDTH,HEIGHT); 
}

mapDisplay::mapDisplay(int lat, int lon, CvImage* startImage)
{	
	GPSState ref;
	ref.lon=lon;
	ref.lat=lat;
	mapDisplay(ref,startImage);
}

void mapDisplay::draw(CvImage* newImage)
{
	lastImage=(IplImage*)newImage;
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX,1.0,1.0,1,CV_AA);
	for(unsigned int i=0;i<GPSPoints.size();i++)
		cvCircle(lastImage,GPSPoints[i],1,cvScalar(0,255,0));
	for(unsigned int i=0;i<additionalPoints.size();i++)
		cvCircle(lastImage,additionalPoints[i],1,cvScalar(0,0,255));
	cvShowImage("Field Map",&lastImage);
		
}	

void mapDisplay::addPointToDraw(GPSState newPoint)
{
	double xGPS=newPoint.lon-referencePoint.lon;
	double yGPS=newPoint.lat-referencePoint.lat;
	lambert_distance_xy(newPoint,referencePoint,&xGPS,&yGPS);
	CvPoint toAdd=cvPoint ((int)(xGPS/meters_per_pixel),(int)(yGPS/meters_per_pixel));
	additionalPoints.push_back(toAdd);
}

void mapDisplay::addGPSPointToDraw(GPSState newGPSPoint)
{
	double xGPS=newGPSPoint.lon-referencePoint.lon;
	double yGPS=newGPSPoint.lat-referencePoint.lat;;
	lambert_distance_xy(newGPSPoint,referencePoint,&xGPS,&yGPS);
	CvPoint toAdd=cvPoint ((int)(xGPS/meters_per_pixel),(int)(yGPS/meters_per_pixel));
	GPSPoints.push_back(toAdd);

}

void mapDisplay::addPointToDraw(double lat, double lon)
{
	GPSState newPoint;
	newPoint.lon=lon;
	newPoint.lat=lat;
	addPointToDraw(newPoint);
}

void mapDisplay::addGPSPointToDraw(double lat, double lon)
{
	GPSState newPoint;
	newPoint.lon=lon;
	newPoint.lat=lat;
	addGPSPointToDraw(newPoint);
}
