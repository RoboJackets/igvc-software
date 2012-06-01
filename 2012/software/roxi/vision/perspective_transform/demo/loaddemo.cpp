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
static int ct=0;
GLuint cameraImageTextureID;


#include "mouse_callback.cpp"
#include "util.cpp"//utility functions
#include "firewire_harness.cpp"



void dotransform(){
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
  	
  	double flat[16]=			{1	,0		,0		,0 \
							,0	,1		,0		,0 \
							,0	,0		,0		,0 \
							,0	,0 		,0		,1};//doesn't invert w so that reverse transformations wind up in the right half space of clip coordinates(no clip condition is -w_clip<=(x,y,z)_clip<=w_clip, ie. -1<=NDC<=1 && w_clip>0
	double flatn[16]=		{-1	,0		,0		,0 \
							,0	,-1		,0		,0 \
							,0	,0		,0		,0 \
							,0	,0 		,0		,-1};//inverts w so that reverse transformations wind up in the right half space of clip coordinates(no clip condition is -w_clip<=(x,y,z)_clip<=w_clip, ie. -1<=NDC<=1 && w_clip>0	
  	
  	//load map matrix							
	if(map_matrix){
 		//cvSave( "PerspectiveMapMatrix.xml", map_matrix  );
   	cvReleaseMat(&map_matrix);
	}
	map_matrix=(CvMat*)cvLoad( "PerspectiveMapMatrix.xml" );
   						
  	glMatrixMode (GL_MODELVIEW);
  	
	glLoadMatrixd(gl2cv.data.db);
	
	//
	//setup projection matrix to transform correctly using opencv projection
	//
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();             /* clear the projection matrix */
	
	if (map_matrix&&(cvmGet( map_matrix, 0, 0)>=0)){//I think this chooses the right one??
		glMultMatrixd(flat);
	}else{
		glMultMatrixd(flatn);
	}
   
   //load up so that persp transform from opencv's output maps back to gl coords
   double cv2gldat[16]; 
   CvMat cv2gl=cvMat(4,4,CV_64FC1,cv2gldat);
   cvInvert(&gl2cv,&cv2gl);	//invert to get the reverse transformation
   glMultMatrixd(cv2gl.data.db);
  
   //tack on the perspective transform from opencv
   
	
	multMat2d(map_matrix);
	cout<<"Matrix Loaded"<<endl;
   
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
	/******* Undistort Image *******/
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
		firstrun=0;
	}
	cvRemap( tmp, MainImage, mapx, mapy ); // undistort image
	/*******/
	
	shownow("mainWin",MainImage);	   
	
	if (waitKey(1)>=0)exit(0);               
}



void init(void) 
{
	// setup opengl stuff
   glClearColor (0.0, 0.0, 0.0, 0.0);
   glShadeModel (GL_FLAT);
   glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(1, &cameraImageTextureID);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, cameraImageTextureID);
	glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

}

void display(void)
{
	cvDisplay();// get a frame 
	dotransform();//load saved persp transform
	//load image to texture
	glTexImage2D(	GL_TEXTURE_RECTANGLE_ARB, 
					0, 
					GL_RGB, 
					MainImage->width, 
					MainImage->height, 
					0, 
					GL_BGR, 
					GL_UNSIGNED_BYTE, 
					MainImage->imageData);

   float xstep=1;
   float ystep=1;
   for(float x=-1;x<1;x+=xstep){
   	for(float y=-1;y<1;y+=ystep){
   		//
		   float xt=((x)/2+.5)*(MainImage->width);
		   float yt=(1-((y)/2+.5))*(MainImage->height);//textures load upside down on opengl
		   float xtstep=((xstep)/2+.5)*(MainImage->width)/2;
		   float ytstep=-1*((ystep)/2+.5)*(MainImage->height)/2;
		   //
		   /*
		   float xt=x;
		   float yt=y;
		   float xtstep=xstep;
		   float ytstep=ystep;
		   */
		   glColor3f ((x/2+.5)*(((float)(ct%2==0))*.5+.5), (((float)(ct%2==0))*.5+.5), (y/2+.5)*(((float)(ct%2==0))*.5+.5));
			glBegin(GL_TRIANGLE_STRIP);
			
			glTexCoord2f(xt,yt+ytstep);
			glVertex2f(x,y+ystep);
			
			glTexCoord2f(xt+xtstep,yt+ytstep);
			glVertex2f(x+xstep,y+ystep);
			
			glTexCoord2f(xt,yt);
			glVertex2f(x,y);
			
			glTexCoord2f(xt+xtstep,yt);
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
	//setup camera
	useFirewire=connectToCamera()?0:1;
	if (!useFirewire){
		camera=cvCaptureFromCAM(0);
	}

	//setup opencv stuff
	cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE); 
	cvMoveWindow("mainWin", 800, 100);
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
