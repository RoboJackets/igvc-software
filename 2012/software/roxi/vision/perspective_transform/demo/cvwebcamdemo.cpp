#include <cv.h>
#include <cvaux.h>
#include <highgui.h>  
#include <iostream>


using namespace cv;
using namespace std;
//This demo shows basic opencv stuff as well
//as demoing how to do perspective transform stuff
//Requires a default webcam and face.png



void shownow(const char* name,const Mat& m){
imshow(name,m);
waitKey(1);
}


int main(){
	
		Mat imgm=imread("face.png",1);
		IplImage img;
		IplImage *i=0;
		IplImage *i2=0;
		IplImage *i3=0;
		img=imgm;//demo create IplImage from a Mat
		Mat img2(&img);//demo convert back
		cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE); 
		cvMoveWindow("mainWin", 100, 100);
		imshow("mainWin", imgm );
		CvCapture* cap=cvCaptureFromCAM(0);//capture from default camera
		//CvCapture* cap=cvCreateCameraCapture(0);//deprecated I think
		




		while(1){
			i=cvQueryFrame(cap);//capture camera frame, but not allowed to release
			i2=cvCloneImage(i);//copy i's header and data
			//cvReleaseImage(&i2);//dealloc i's data
			//switch rapidly between camera image and test image
			shownow("mainWin",i2);
			
			cvReleaseImage(&i3);
			i3=cvCloneImage(i2);//copy i's header and data
			cvResize(&img,i3);//resize test pic to frame
			shownow("mainWin",i3);//draw test pic
			
			if(waitKey(1)>0) break;//process events and quit on key press
			cvReleaseImage(&i2);
		}
	
	
	cvDestroyAllWindows();
	cvReleaseImage(&i2);
	cvReleaseImage(&i3);
	return 0;
}
