#include <cv.h>
#include <cvaux.h>
#include <highgui.h>  
#include <iostream>

using namespace cv;
using namespace std;

#include "mouse_callback_line.cpp"



//This demo shows basic opencv stuff as well
//as demoing how to do perspective transform stuff
//Requires a default webcam

static IplImage *MainImage;

void shownow(const char* name,const Mat& m){
	imshow(name,m);
	waitKey(1);
}

void dotransform(){
	if (ptnum!=4)
		return;
	CvMat*  map_matrix=cvCreateMat( 3, 3, CV_32FC1 );
	CvPoint2D32f src[4];
	double sc=100;
	CvPoint2D32f dst[4]={ cvPoint2D32f(0.,sc),cvPoint2D32f(sc,sc),cvPoint2D32f(sc,0.),cvPoint2D32f(0.,0.)};
	
	//Load src and dst points
	for(int i=0;i<4;i++){
		src[i]=cvPointTo32f(pt[i]);
	}
	
	cvGetPerspectiveTransform( src,dst,
                               map_matrix );
   IplImage* tmp=cvCloneImage(MainImage);

  	cvWarpPerspective( MainImage, tmp, map_matrix,
                        CV_INTER_LINEAR&(!CV_WARP_FILL_OUTLIERS),	//flags
                        cvScalarAll(0) );									//fillval
   cvReleaseImage(&MainImage);
   MainImage=cvCloneImage(tmp);
   cvReleaseImage(&tmp);
   
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
			dotransform();//use the four points clicked by user to do a perspective transform 
			img=MainImage;//Mainimage was altered by tansform
			
			shownow("mainWin",img);

			cvReleaseImage(&img);
		}
	
	
	cvDestroyAllWindows();
	return 0;
}
