#include "mapDisplay.hpp"

mapDisplay::mapDisplay(GPSState reference, cvImage startImage)
{
	lastImage=(IplImage)startImage;
	referencePoint=reference;
	cvNamedWindow("Field Map",CV_WINDOW_AUTOSIZE);
	cvMoveWindow("Field Map",X_COORD_ON_SCREEN ,Y_COORD_ON_SCREEN);
	cvResizeWindow("Field Map",WIDTH,HEIGHT); 
}

mapDisplay::draw(cvImage newImage)
{
	lastImage=(IplImage)startImage;
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX,1.0,1.0,1,CV_AA);
	for(int i=0;i<GPSPoints.size();i++)
		Circle(lastImage,GPSPoints[i],1,CvScalar(0,255,0));
	for(int i=0;i<additionalPoints.size();i++)
		Circle(lastImage,dditionalPoints[i],1,CvScalar(0,0,255));
	cvShowImage("Field Map",&lastImage);
		
}	

mapDisplay::addPointToDraw(GPSState newPoint)
{
	double xGPS;
	double yGPS;
	lambert_distance_xy(newPoint,referencePoint,&xGPS,&yGPS);
	cvPoint toAdd((int)(xGPS/meters_per_pixel),(int)(yGPS/meters_per_pixel));
	additionalPoints.push_end(toAdd);
}

mapDisplay::addGPSPointToDraw(GPSState newGPSPoint)
{
	double xGPS;
	double yGPS;
	lambert_distance_xy(newGPSPoint,referencePoint,&xGPS,&yGPS);
	cvPoint toAdd((int)(xGPS/meters_per_pixel),(int)(yGPS/meters_per_pixel));
	GPSPoints.push_end(toAdd);

}

mapDisplay::addPointToDraw(double lat, double lon)
{
	GPSState newPoint;
	givenStates.lon=lon;
	givenStates.lat=lat;
	double xGPS;
	double yGPS;
	lambert_distance_xy(newPoint,referencePoint,&xGPS,&yGPS);
	cvPoint toAdd((int)(xGPS/meters_per_pixel),(int)(yGPS/meters_per_pixel));
	additionalPoints.push_end(toAdd);
}

mapDisplay::addGPSPointToDraw(double lat, double lon)
{
	GPSState newPoint;
	givenStates.lon=lon;
	givenStates.lat=lat;
	double xGPS;
	double yGPS;
	lambert_distance_xy(newPoint,referencePoint,&xGPS,&yGPS);
	cvPoint toAdd((int)(xGPS/meters_per_pixel),(int)(yGPS/meters_per_pixel));
	GPSPoints.push_end(toAdd);
}
