#include "PjMat.h"
#include <math.h>
#include <stdio.h>


/* IMPORTANT CAM ANGLE *********/
// (how much up from 0 we are, in radians)
// 1.25 default from 2007
// update: current measured guppy angle is ? for 2009
#define OFFANGLE  0.93  // 1.1~=63deg
// need to measure stuff below for guppy!
//  (remember to adjust nav_path__view_distance_multiplier when changing this!)
/* *************************** */

/* use roi to shift camera view down to only see bottom of image */
//  0 - if using narrow angle lens
//  1 - if using  wide  angle lens
#define USE_ROI 1

//utility macros
#define max(a,b) ((a>b)?a:b)
#define min(a,b) ((a<b)?a:b)

//inputs - the guppy camera uses 640x480 resolution
#define HFOV (0.85)           // half angle value in radians | (0.85~=48.7deg)-new (0.7854~=45deg)-old
#define VFOV (HFOV*0.75)		// aspect ratio == 0.75 == 480/640
#define u (HFOV/2)
#define v (VFOV/2)

volatile float h = 1;	    // height of camera
volatile float x = -M_PI/2+OFFANGLE; // see above
volatile float y = M_PI;
volatile float z = M_PI;   //euler rotations about given axes

int j=0;
GLdouble xscale,yscale,translateX, translateY;

void findmaxview(GLdouble* n);
GLdouble m[16];

void setPjMat()
{
	//int i;
	float tu,tv;
	float sx,sy,sz,cx,cy,cz;
	//float translateX, translateY;

	tu=tan(u);
	tv=tan(v);
	sx=sin(x);
	sy=sin(y);
	sz=sin(z);
	cx=cos(x);
	cy=cos(y);
	cz=cos(z);

	GLdouble n[16]={cy*cz*tu,(sx*sz-cx*cz*sy)*tu,0,((cx*cx)*(cz*cz)*sy-cx*cz*sx*((sy*sy)+1)*sz+(sx*sx*(sz*sz)-1)*sy)*tu/(h*(cx*sy*sz+cz*sx)),\
					-cy*sz*tv,(cx*sy*sz+cz*sx)*tv,0,(sx*sy*sz-cx*cz)*tv/h,\
					0,0,0,0,\
					sy,cx*cy,0,cy*sx/h
				   };

	findmaxview(&n[0]);

	glMatrixMode(GL_PROJECTION);

	glLoadIdentity ();
	glOrtho (-xscale, xscale, -yscale, yscale, -10.0, 10.0);
	glTranslated(-translateX, -translateY, 0);
	glMultMatrixd(&n[0]);

	//currx=botx-translateX;
	//curry=boty-translateY;
	if (!j)
	{
		printf("frame translated by: x:%lf ,y:%lf \n",(translateX), (translateY));
		j++;
	}
}

void findmaxview(GLdouble* n)
{
	GLdouble maxx,minx,maxy,miny;
	GLdouble ly,ry,ty,by,_try,tly,bry,bly; //define the principal maximum values
	GLdouble lx,rx,tx,bx,trx,tlx,brx,blx; //define the principal maximum values
	//XXX:xi,yi should really be calculated by Genpoints and written as a variable,but now i don't have time

#	define xi (.96)
#	define yi (.85)
#	define xtran (n[12]/n[15])
#	define ytran (n[13]/n[15])

	rx=(n[0]*xi+n[12])/(n[3]*xi+n[15]);
	ry=(n[1]*xi+n[13])/(n[3]*xi+n[15]);

	lx=(-n[0]*xi+n[12])/(-n[3]*xi+n[15]);
	ly=(-n[1]*xi+n[13])/(-n[3]*xi+n[15]);

	tx=(n[4]*yi+n[12])/(n[7]*yi+n[15]);
	ty=(n[5]*yi+n[13])/(n[7]*yi+n[15]);

	bx=(-n[4]*yi+n[12])/(-n[7]*yi+n[15]);
	by=(-n[5]*yi+n[13])/(-n[7]*yi+n[15]);

	trx=(n[0]*xi+n[4]*yi+n[12])/(n[3]*xi+n[7]*yi+n[15]);
	_try=(n[1]*xi+n[5]*yi+n[13])/(n[3]*xi+n[7]*yi+n[15]);

	tlx=(-n[0]*xi+n[4]*yi+n[12])/(-n[3]*xi+n[7]*yi+n[15]);
	tly=(-n[1]*xi+n[5]*yi+n[13])/(-n[3]*xi+n[7]*yi+n[15]);

	brx=(n[0]*xi-n[4]*yi+n[12])/(n[3]*xi-n[7]*yi+n[15]);
	bry=(n[1]*xi-n[5]*yi+n[13])/(n[3]*xi-n[7]*yi+n[15]);

	blx=(-n[0]*xi-n[4]*yi+n[12])/(-n[3]*xi-n[7]*yi+n[15]);
	bly=(-n[1]*xi-n[5]*yi+n[13])/(-n[3]*xi-n[7]*yi+n[15]);

	/****** 8-Neighbors *********/
	maxy=max(max(max(max(max(max(max(					\
										  ly,ry),ty),by),_try),tly),bry),bly);
	maxx=max(max(max(max(max(max(max(					\
										  lx,rx),tx),bx),trx),tlx),brx),blx);
	miny=min(min(min(min(min(min(min(					\
										  ly,ry),ty),by),_try),tly),bry),bly);
	minx=min(min(min(min(min(min(min(					\
										  lx,rx),tx),bx),trx),tlx),brx),blx);

	/****** 4-Neighbors *********/
	/*maxy=max(max(max(					\
			ly,ry),ty),by);
	maxx=max(max(max(					\
			lx,rx),tx),bx);
	miny=min(min(min(					\
			ly,ry),ty),by);
	minx=min(min(min(					\
			lx,rx),tx),bx);*/

	yscale=(maxy-miny)/2;
	xscale=(maxx-minx)/2;
	translateX=minx+xscale;
	translateY=miny+yscale;

#if USE_ROI
	//start of region of interest code
	yscale=yscale/2;
	xscale=xscale/2;
	//translateX=minx+xscale; // always commented out
	translateY=miny+3.0*yscale;  // 3.1 is magical
	//end roi code
#endif

#	undef xtran
#	undef ytran
#	undef yi
#	undef xi

}


GLdouble* getPjMat()
{
	int i;
	float tu,tv;
	float sx,sy,sz,cx,cy,cz;
	//symbols to make the matrix easier to type
	tu=tan(u);
	tv=tan(v);
	sx=sin(x);
	sy=sin(y);
	sz=sin(z);
	cx=cos(x);
	cy=cos(y);
	cz=cos(z);
	//old broken matrix
	/*GLdouble m[16]={	cx*cy*tu/(cx*cz+sx*sy*sz),				0,		0,	-(cx*cz*sy+sx*sz)/(cx*cz+sx*sy*sz),	\
					/*0,0,0,0,/\
					sy*tu/(cx*cz+sx*sy*sz),					0,		0,	cy*cz/(cx*cz+sx*sy*sz),				\
					0,										0,		0,	0,								\
					tu/h*(cz*sx*sy-cx*sz)/(cx*cz+sx*sy*sz),	tv/h,	0,	cy*sx/h/(cx*cz+sx*sy*sz)			\
					};*/
	//new shiny matrix
	//translating to 0,0
	/*GLdouble n[16]={cy*cz*tu,-cy*sz*tv,0,sy,\
					(sx*sz-cx*cz*sy)*tu,(cx*sy*sz+cz*sx)*tv,0,cx*cy,\
					0,0,0,0,\
					((cx*cx)*(cz*cz)*sy-cx*cz*sx*((sy*sy)+1)*sz+(sx*sx*(sz*sz)-1)*sy)*tu/(h*(cx*sy*sz+cz*sx)),(sx*sy*sz-cx*cz)*tv/h,0,cy*sx/h
					};*/
	//nontranslating
	GLdouble n[16]={cy*cz*tu,-cy*sz*tv,0,0,\
					(sx*sz-cx*cz*sy)*tu,(cx*sy*sz+cz*sx)*tv,0,0,\
					0,0,0,0,\
					((cx*cx)*(cz*cz)*sy-cx*cz*sx*((sy*sy)+1)*sz+(sx*sx*(sz*sz)-1)*sy)*tu/(h*(cx*sy*sz+cz*sx)),(sx*sy*sz-cx*cz)*tv/h,0,1
				   };


	for (i=0; i<16; i++)
	{
		m[i]=n[i];
	}
	GLdouble *m2=m;//convert to pointer
	return m2;
}
