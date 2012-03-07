#include <cv.h>
#include <cvaux.h>
#include <highgui.h>  
#include <iostream>

using namespace cv;
using namespace std;

#include "mouse_callback_line.cpp"



//This demo shows basic opencv stuff as well
//as demoing how to do perspective transform stuff
//Requires a default webcam and face.png

static IplImage *MainImage;

void shownow(const char* name,const Mat& m){
	imshow(name,m);
	waitKey(1);
}

//Draws lines on main image
void drawlines(){
	for(int i=1;i<ptnum;i++)if(ptnum>=2){
		cout<<"line "<<i<<endl;
		cvLine( MainImage, pt[i], pt[i-1], cvScalar(0xff,0x00,0x00), 1, 8, 0);
	}
	if (ptnum>3){
		cvLine( MainImage, pt[3], pt[0], cvScalar(0xff,0x00,0x00), 1, 8, 0);
	}
}

int main(){
	
		Mat imgm=imread("face.png",1);
		IplImage *img;
		IplImage *tmp=0;


		cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE); 
		cvMoveWindow("mainWin", 100, 100);
		CvCapture* cap=cvCaptureFromCAM(0);//capture from default camera
		cvSetMouseCallback( "mainWin", mouse_callback, (void*) MainImage);


		while(1){
			tmp=cvQueryFrame(cap);	//capture camera frame, but not allowed to release because 
											//it is a pointer to an internal buffer
			img=cvCloneImage(tmp);	//copy i's header and data to my buffer
			
			MainImage=img;
			drawlines();
			
			shownow("mainWin",img);

			cvReleaseImage(&img);
		}
	
	
	cvDestroyAllWindows();
	return 0;
}
