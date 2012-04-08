#include <GL/glut.h>
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>  
#include <iostream>

using namespace cv;
using namespace std;

CvCapture* camera=cvCaptureFromCAM(0);//capture from default camera
static IplImage *MainImage;
CvMat*  map_matrix=0;

#include "mouse_callback.cpp"
#include "util.cpp"//utility functions



void dotransform(){
	if (ptnum!=4)
		return;
	map_matrix=cvCreateMat( 3, 3, CV_32FC1 );
	CvPoint2D32f src[4];
	double sc=100;
	//upper left corner of mapped square
	CvPoint2D32f ul=cvPoint2D32f(MainImage->width/2-sc/2,MainImage->height/2-sc/2);
	
	//the four points to map the clicked ones to
	CvPoint2D32f dst[4]={ add(cvPoint2D32f(0.,sc),ul),add(cvPoint2D32f(sc,sc),ul),add(cvPoint2D32f(sc,0.),ul),add(cvPoint2D32f(0.,0.),ul)};
	
	//Load src and dst points
	for(int i=0;i<4;i++){
		src[i]=cvPointTo32f(pt[i]);
	}
	
	cvGetPerspectiveTransform( src,dst,
                               map_matrix );//put the transform into mapmatrix
                               
   printMat(map_matrix);
   IplImage* tmp=cvCloneImage(MainImage);

  	cvWarpPerspective( MainImage, tmp, map_matrix,
                        CV_INTER_AREA&(!CV_WARP_FILL_OUTLIERS),	//flags
                        cvScalarAll(0) );									//fillval
   cvReleaseImage(&MainImage);
   MainImage=tmp;
   
}

void cvDisplay(){
	IplImage *tmp,*img;
	tmp=cvQueryFrame(camera);	//capture camera frame, but not allowed to 
			                  //release because it is a pointer to an internal buffer
	try{				
		cvReleaseImage(&MainImage);			
		MainImage=cvCloneImage(tmp);	//copy i's header and data to my buffer
	}catch(...){							//image data was corrupted, ignore it and 
										//throw in a fake
										//as a workaruound
		cout<<"Ignoring presumed bad jpeg"<<endl;
		MainImage=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3); 
	}
	
	drawlines();
	dotransform();//alters mainimage
	
	
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
	cvDisplay();
	glutReshapeWindow
              (MainImage->width
              , MainImage->height
              );


   glClear (GL_COLOR_BUFFER_BIT);
   glColor3f (1.0, 1.0, 1.0);
   glLoadIdentity ();             /* clear the matrix */
   if(map_matrix) loadMat(map_matrix);
           /* viewing transformation  */
   //gluLookAt (0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
   //glScalef (1.0, 2.0, 1.0);      /* modeling transformation */ 
   //glutSolidCube (2.0*499.01/500);
   
   glBegin(GL_TRIANGLE_STRIP);
   glVertex2f(0,1);
   glVertex2f(1,1);
	glVertex2f(0,0);
	glVertex2f(1,0);
   glEnd();
   glBegin(GL_TRIANGLE_STRIP);
   glVertex2f(0,-1);
   glVertex2f(-1,-1);
	glVertex2f(0,0);

   glEnd();
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
