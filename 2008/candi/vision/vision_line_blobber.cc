#include "vision_line_blobber.h"
#include "Graphics.h"
#include "vision.h"				// for visRaw
#include "vision_color.h"		// for pixelIsWhite

#include <stdlib.h>
#include <stdio.h>
#include <math.h>


/* I refer to points as if cartesian for simplicity in code,	*
 * output is not affected by this abuse.						*/
 
#define max(a,b) (((a)>(b))?(a):(b))
#define min(a,b) (((a)<(b))?(a):(b))

Line<int> whitelines[MAX_N_LINES];		//array of the white lines we've found
int numwhitelines=0;					//number of lines in the line array
#define numlines numwhitelines

Line<int> inferredlines[MAX_N_LINES];	//array of the white lines we've found
int numinferredlines=0;					//number of lines in the line array

Line<int> dashedlines[MAX_N_LINES];		//array of the white lines we've found
int numdashedlines=0;					//number of lines in the line array

//internal use
static Buffer2D<bool> img;
Buffer2D<bool> whiteFilterMask;
Buffer2D<Pixel> paulBlob;

static int w;
static int h;

static void findat(int x,int y);
static void figureDashes();
static int findlowestleftline();
static int findlowestrightline();

// functions!
void visBlobLines(){
	img.copyFrom(pixelIsWhite);
	
	numwhitelines=0;
	w=img.width;
	h=img.height;
	
	// Black out the edges to avoid buffer overruns in subsequent code
	for (int x=0; x<w; x++) img[x] = 0;
	for (int x=0, off=(h-1)*w; x<w; x++) img[off] = 0;
	for (int y=0, off=0; y<h; y++, off+=w) img[off] = 0;
	for (int y=0, off=w-1; y<h; y++, off+=w) img[off] = 0;
	
	for(int x=0;x<w;x++){
		for(int y=0;y<h;y++) {
			/*eight neighbor check					*/
			/*done in order of best short circuit	*/
			if(img.data[x+y*w]&&				
				img.data[x+1+y*w]&&				
				img.data[x-1+y*w]&&		
				img.data[x+(y+1)*w]&&	
				img.data[x+1+(y+1)*w]&&	
				img.data[x-1+(y+1)*w]&&	
				img.data[x+(y-1)*w]&&	
				img.data[x+1+(y-1)*w]&&	
				img.data[x-1+(y-1)*w])
			{
				//if (numwhitelines<20) {
					//printf("\n%d",numwhitelines);
					findat(x,y);
				//}
			}		
				
		}
	}
	
	//figureDashes();
	whiteFilterMask.copyFrom(img);
	// Draw blob view
	
	
	paulBlob.copyFrom(visRaw);
	Graphics g(&paulBlob);
	g.setColor(Pixel(200, 0, 0));	// dark red
	for(int i=0;i<numwhitelines;i++){
		Line<int> curLine = whitelines[i];
		for (int dx=-1; dx<=1; dx++) {
			g.drawLine(curLine.a.x + dx, curLine.a.y, curLine.b.x + dx, curLine.b.y);
		}
	}
}

void findat(int x,int y) {
	
	
	static Point2D<int>* plst=0;		// pixel list
	static int plsz=0;					// pixel list size

	Point2D<int> xmax(x, y);
	Point2D<int> xmin(x, y);
	Point2D<int> ymax(x, y);
	Point2D<int> ymin(x, y);
	const int width = img.width;
	//const int h=img.height;
	
	if (img.numElements()/8 != plsz) {
		plst = new Point2D<int>[img.numElements()/8];
		plsz = img.numElements()/8;
	}
		
	// lets blobberize!
	int cp=0;		//current point index
	int np=1;		//num points so far found, starts with 1
	
	//read in initial point
	plst[cp].x=x;	
	plst[cp].y=y;
	
	img.data[plst[cp].x+plst[cp].y*width]=0;		//clear initial point
	for(;cp<np;cp++){						//run till out of points
		//clear and record 4 neighbors
		{
			//cache base x and y (hopefully in registers)			 
				int cx=plst[cp].x;
				int cy=plst[cp].y;

				
			//variable for index
				int index = 0;
			
			//white top
			{
				index = cx+(cy+1)*width;
				if(img.data[index]){
					img.data[index]=0;	//clear it
					if(cy+1>ymax.y){				//could be new ymax,store if so
						ymax.y=cy+1;
						ymax.x=cx;	
					}
					plst[np].y=cy+1;			//add to found list
					plst[np].x=cx;
					np++;
				}
			}
			//white right
			{
				index = cx+1+(cy)*width;
				if(img.data[index]){		
					img.data[index]=0;	//clear it
					if(cx+1>xmax.x){				//could be new xmax,store if so
						xmax.y=cy;
						xmax.x=cx+1;
					}	
					plst[np].y=cy;			//add to found list
					plst[np].x=cx+1;
					np++;
				}
			}
			//white bottom
			{
				index = cx+(cy-1)*width;
				if(img.data[index]){
					img.data[index]=0;	//clear it
					if(cy-1<ymin.y){	//could be new ymin,store if so
						ymin.y=cy-1;
						ymin.x=cx;	
					}
					plst[np].y=cy-1;			//add to found list
					plst[np].x=cx;
					np++;
				}
			}
			//white left
			{
				index = cx-1+(cy)*width;
				if(img.data[index]){		
					img.data[index]=0;	//clear it
					if(cx-1<xmin.x){	//could be new xmin,store if so
						xmax.y=cy;
						xmax.x=cx-1;
					}	
					plst[np].y=cy;			//add to found list
					plst[np].x=cx-1;
					np++;
				}
			}
		}
		
		//printf("\n%d,%d",np,cp);
		if (np>=plsz-8){				//in danger of running out of memory next pass
			printf("too many white pixels - ignoring blob\n");
			break;				// we have selected ~1/8 of the image!
		}
	}
	//we have a line candidate!
	//is it big enough?
	if ((np < MIN_LINE_BLOB_POINTS)||(numwhitelines==MAX_N_LINES))
		return;	//if not then dump it
	
	/* we will assume that the points with the
	 * maximum dist between them are the endpoints.
	 * we use square distance for speed*/
	/* ______
	   |\  /|
	   | \/ | 6 comparisons
	   | /\ |
	   |/  \|
	   ------  */
	#define sqr(x) ((x)*(x))
	#define dist(p1, p2) (sqr(p1.x-p2.x) + sqr(p1.y-p2.y))
	int l1 = dist(xmax, xmin);
	int l2 = dist(ymax, ymin);
	int l3 = dist(ymax, xmax);
	int l4 = dist(ymin, xmin);
	int l5 = dist(ymax, xmin);
	int l6 = dist(xmax, ymin);
	int ans = max(max(max(max(max(l1,l2),l3),l4),l5),l6);
	if (ans==l1){
		whitelines[numwhitelines].a=xmax;
		whitelines[numwhitelines].b=xmin;
	}else
	if (ans==l2){
		whitelines[numwhitelines].a=ymax;
		whitelines[numwhitelines].b=ymin;
	}else
	if (ans==l3){
		whitelines[numwhitelines].a=ymax;
		whitelines[numwhitelines].b=xmax;
	}else
	if (ans==l4){
		whitelines[numwhitelines].a=xmin;
		whitelines[numwhitelines].b=ymin;
	}else
	if (ans==l5){
		whitelines[numwhitelines].a=ymax;
		whitelines[numwhitelines].b=xmin;
	}else
	if (ans==l6){
		whitelines[numwhitelines].a=xmax;
		whitelines[numwhitelines].b=ymin;
	}else printf("VisLineBlob:error finding line bounds");
	#undef dist
	#undef sqr
	
	//put this here to be thread safe(ie. it doesn't exist till it's done)
	numwhitelines++;
}

static int findlowestleftline(){
	#define wl whitelines
	#define sqr(x) ((x)*(x))
	#define dist(p1, p2) (sqr(p1.x-p2.x) + sqr(p1.y-p2.y))
	int lowestleftlineindex=-1; //<-not found!
	Point2D<int> me(w/2,h);
	Point2D<int> lowestleftlinept(0,0); //<-worst case
	
	for(int i=0;i<numwhitelines;i++){
		if(wl[i].a.x<w/2||wl[i].b.x<w/2){
			if(wl[i].a.y > lowestleftlinept.y){
				lowestleftlineindex=i;
				lowestleftlinept=wl[i].a;
			}
			if(wl[i].b.y > lowestleftlinept.y){
				lowestleftlineindex=i;
				lowestleftlinept=wl[i].b;
			}
		}
	}
	return lowestleftlineindex;
	#undef wl 
	#undef sqr
	#undef dist
}

static int findlowestrightline(){
	#define wl whitelines
	#define sqr(x) ((x)*(x))
	#define dist(p1, p2) (sqr(p1.x-p2.x) + sqr(p1.y-p2.y))
	int lowestrightlineindex=-1; //<-not found!
	Point2D<int> me(w/2,h);
	Point2D<int> lowestrightlinept(0,0); //<-worst case
	
	for(int i=0;i<numwhitelines;i++){
		if(wl[i].a.x<w/2||wl[i].b.x<w/2){
			if(wl[i].a.y > lowestrightlinept.y){
				lowestrightlineindex=i;
				lowestrightlinept=wl[i].a;
			}
			if(wl[i].b.y > lowestrightlinept.y){
				lowestrightlineindex=i;
				lowestrightlinept=wl[i].b;
			}
		}
	}
	return lowestrightlineindex;
	#undef wl 
	#undef sqr
	#undef dist
}

bool dashcheckleft(Line<int> lowleft){
	if(lowleft.a.y<LINE_MARGIN && lowleft.b.y>h-LINE_MARGIN){
		return true;
	}if(lowleft.b.y<LINE_MARGIN && lowleft.a.y>h-LINE_MARGIN){
		return true;
	}
	return false;
}

bool dashcheckright(Line<int> lowright){
	if(lowright.a.y<LINE_MARGIN && lowright.b.y>h-LINE_MARGIN){
		return true;
	}if(lowright.b.y<LINE_MARGIN && lowright.a.y>h-LINE_MARGIN){
		return true;
	}
	return false;
}

void findangles(double* angle){
	for(int i=0;i<numwhitelines;i++){
		Point2D<int> p=whitelines[i].a-whitelines[i].b;
		angle[i]=atan(-p.y/p.x);//flipped y coord
	}
}

static void figureDashes() {
	#define sqr(x) ((x)*(x))
	#define dist(p1, p2) (sqr(p1.x-p2.x) + sqr(p1.y-p2.y))
	
	//first, do we need them?
	findlowestleftline();
	findlowestrightline();
	
	bool haveleft, haveright;
	Line<int> lowleft, lowright;
	if(haveleft=(findlowestleftline()!=-1))
		lowleft=whitelines[findlowestleftline()];
	if (haveright=(findlowestrightline()!=-1))
		lowright=whitelines[findlowestrightline()];
	
	double angle[numlines];
	bool lgood=0;
	bool rgood=0;
	
	//check for ideal non dashed cases
	
	if(numlines==2){
		if (haveleft)
		lgood=dashcheckleft(lowleft);
		if (haveright)
		rgood=dashcheckright(lowright);
	}
	if(numlines==1) return;
	if(numlines==2 && lgood && rgood) return;//no need for dashfill
	findangles(angle);
	
	Point2D<int> high,closest(10000,10000);
	high=(lowleft.a.y<lowleft.b.y) ? lowleft.a : lowleft.b;
	if(haveleft && !lgood){
		for(int i=0;i<numlines;i++){
			if(whitelines[i].a!=lowleft.a){ //xxx
				closest=dist(whitelines[i].a,high)<dist(closest,high)?whitelines[i].a:closest;				
				closest=dist(whitelines[i].b,high)<dist(closest,high)?whitelines[i].b:closest;
			}
		}
		dashedlines[numdashedlines].a=high;
		dashedlines[numdashedlines].b=high;
	}
	
	#undef sqr
	#undef dist		
}
