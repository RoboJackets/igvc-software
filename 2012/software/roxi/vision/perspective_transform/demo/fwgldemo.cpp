#include <GL/glut.h>
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>  
#include <iostream>

using namespace cv;
using namespace std;

CvCapture* camera=NULL;//capture from default camera
static IplImage *MainImage;
CvMat*  map_matrix=0;
int useFirewire;

#include "mouse_callback.cpp"
#include "util.cpp"//utility functions
#include "firewire_harness.cpp"



void dotransform(){
	if (ptnum!=4)
		return;
	map_matrix=cvCreateMat( 3, 3, CV_32FC1 );
	CvPoint2D32f src[4];
	double sc=200;
	//upper left corner of mapped square
	CvPoint2D32f ul=cvPoint2D32f(MainImage->width/2-sc/2,MainImage->height-sc-1);
	
	//the four points to map the clicked ones to
	CvPoint2D32f dst[4]={ add(cvPoint2D32f(0.,sc),ul),add(cvPoint2D32f(sc,sc),ul),add(cvPoint2D32f(sc,0.),ul),add(cvPoint2D32f(0.,0.),ul)};
	
	//Load src and dst points
	for(int i=0;i<4;i++){
		src[i]=cvPointTo32f(pt[i]);
	}
	
	cvGetPerspectiveTransform( src,dst,
                               map_matrix );//put the transform into mapmatrix
                               
   cvSave( "PerspectiveMapMatrix.xml", map_matrix  );
   
                               
   //printMat(map_matrix);//for debug
   IplImage* tmp=cvCloneImage(MainImage);

  	cvWarpPerspective( MainImage, tmp, map_matrix,
                        CV_INTER_AREA&(!CV_WARP_FILL_OUTLIERS),	//flags
                        cvScalarAll(0) );									//fillval
   cvReleaseImage(&MainImage);
   MainImage=tmp;
   
}


//This displays everything in the cv window
void cvDisplay(){
	IplImage *tmp;
	
	//
	//Capture a camera frame and store in MainImage
	//
	
	//capture camera frame (Don't release this one)
	tmp=useFirewire?camera_firewire.ReturnFrame():cvQueryFrame(camera);
	
	
	try{				
		cvReleaseImage(&MainImage);			
		MainImage=cvCloneImage(tmp);	//copy i's header and data to my buffer
	}catch(...){							//image data was corrupted, ignore it and 
										//throw in a fake
										//as a workaruound
		cout<<"Ignoring presumed bad jpeg"<<endl;
		MainImage=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3); 
		tmp=cvCloneImage(MainImage);//mild memory leak
	}
	static int firstrun=1;
	// Load Distortions
	static CvMat *intrinsic = (CvMat*)cvLoad( "Intrinsics.xml" );
	static CvMat *distortion = (CvMat*)cvLoad( "Distortion.xml" );
	static IplImage* mapx;
	static IplImage* mapy;
	if(firstrun){//first time through
		// Build the undistort map that we will use for all subsequent frames
		mapx = cvCreateImage( cvGetSize( MainImage ), IPL_DEPTH_32F, 1 );
		mapy = cvCreateImage( cvGetSize( MainImage ), IPL_DEPTH_32F, 1 );
		cvInitUndistortMap( intrinsic, distortion, mapx, mapy );
	}
	cvRemap( tmp, MainImage, mapx, mapy ); // undistort image

	
	
	
	
	//draw lines between clicked points on MainImage
	drawlines();
	dotransform();//alters MainImage to transform clicked points to desired shape
	
	
	shownow("mainWin",MainImage);	   
	
	if (waitKey(1)>=0)exit(0);               
}















void init(void) 
{
	// setup opengl stuff
   glClearColor (0.0, 0.0, 0.0, 0.0);
   glShadeModel (GL_FLAT);
   

}

void display(void)
{
	cvDisplay();// do opencv calls for warping image
	
	int w=MainImage->width;
	int h=MainImage->height;
	
	// make the gl window right size for holding opencv's output
	glutReshapeWindow(w, h);
	
	glClear (GL_COLOR_BUFFER_BIT);

              
   //put opengl's model output into opencv's coordinate system (you can work this out by hand or trust me)
   //this lets us use a projection matrix straight out of opencv
   double gl2cvdat[16]=	{w-1	,0			,0		,0 \
   							,0		,-h-1		,0		,0 \
   							,0		,0			,1		,0 \
   							,w		,h 		,0		,2};//transposed because opengl is col major
  	CvMat gl2cv=cvMat(4,4,CV_64FC1,gl2cvdat);
   						
  	glMatrixMode (GL_MODELVIEW);
  	
	glLoadMatrixd(gl2cv.data.db);
	
	//
	//setup projection matrix to transform correctly using opencv projection
	//
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();             /* clear the projection matrix */
   
   double flat[16]=	{1	,0			,0		,0 \
							,0	,1			,0		,0 \
							,0	,0			,0		,0 \
							,0	,0 		,0		,1};//doesn't invert w so that reverse transformations wind up in the right half space of clip coordinates(no clip condition is -w_clip<=(x,y,z)_clip<=w_clip, ie. -1<=NDC<=1 && w_clip>0
	double flatn[16]=	{-1,0			,0		,0 \
							,0	,-1		,0		,0 \
							,0	,0			,0		,0 \
							,0	,0 		,0		,-1};//inverts w so that reverse transformations wind up in the right half space of clip coordinates(no clip condition is -w_clip<=(x,y,z)_clip<=w_clip, ie. -1<=NDC<=1 && w_clip>0	
								
	static int ct=0;
	switch((ct++)%2){
		case 0:glMultMatrixd(flat);break;
		case 1:glMultMatrixd(flatn);break;
   }
   
   //load up so that persp transform from opencv's output maps back to gl coords
   double cv2gldat[16]; 
   CvMat cv2gl=cvMat(4,4,CV_64FC1,cv2gldat);
   cvInvert(&gl2cv,&cv2gl);	//invert to get the reverse transformation
   glMultMatrixd(cv2gl.data.db);
  
   //tack on the perspective transform from opencv
   //if(map_matrix) multMat2d(map_matrix);//use the opencv projection matrix
   if(map_matrix){
 		//cvSave( "PerspectiveMapMatrix.xml", map_matrix  );
   	cvReleaseMat(&map_matrix);
   	map_matrix=(CvMat*)cvLoad( "PerspectiveMapMatrix.xml" );
   	multMat2d(map_matrix);
   	cout<<"Matrix Loaded"<<endl;
    	//multMat2d((CvMat*)cvLoad( "PerspectiveMapMatrix.xml" ));//do it from file
	}

   float xstep=.1;
   float ystep=.1;
   for(float x=-1;x<1;x+=xstep){
   	for(float y=-1;y<1;y+=ystep){
		   //draw an easily recognizable test object (light one is flatn)
			glColor3f ((x/2+.5)*(((float)(ct%2==0))*.5+.5), (((float)(ct%2==0))*.5+.5), (y/2+.5)*(((float)(ct%2==0))*.5+.5));
			glBegin(GL_TRIANGLE_STRIP);
			glVertex2f(x,y+ystep);
			glVertex2f(x+xstep,y+ystep);
			glVertex2f(x,y);
			glVertex2f(x+xstep,y);
			glEnd();
		}
	}
				
   glFlush ();
}

void reshape (int w, int h)
{
   glViewport (0, 0, (GLsizei) w, (GLsizei) h); 
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();
   //glFrustum (-1.0, 1.0, -1.0, 1.0, 1.5, 20.0);
   glMatrixMode (GL_MODELVIEW);
}

int main(int argc, char** argv)
{
	cout<<"This program uses the calibrations from ckdemo, please run it if you haven't already"<<endl;
	//setup camera
	useFirewire=connectToCamera()?0:1;
	if (!useFirewire){
		camera=cvCaptureFromCAM(0);
	}

	//setup opencv stuff
	cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE); 
	cvMoveWindow("mainWin", 100, 100);
	cvSetMouseCallback( "mainWin", mouse_callback, (void*) MainImage);
	

	
   glutInit(&argc, argv);
   glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB);
   glutInitWindowSize (500, 500); 
   glutInitWindowPosition (100, 100);
   glutCreateWindow (argv[0]);
   init ();
   glutDisplayFunc(display); 
   glutIdleFunc(display); 
   glutReshapeFunc(reshape);
   glutMainLoop();
   return 0;
}
