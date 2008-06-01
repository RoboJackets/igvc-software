#include <math.h>
#include <string.h>
#include "shape.h"

class staticheap {

  protected:

    int *pixel;  //Pointer to array of pixels stored in heap (1-offset)
    int size;    //Current size of heap
    int maxsize; //Amount of space allocated to pixel and size arrays

    float *dist; //Signed distance array
    int *bckptr; //Array of backpointers

  public:

    staticheap(float *dist,int *bckptr,int maxsize) {
      if (maxsize<0) maxsize=0;
      this->dist=dist; this->bckptr=bckptr; this->maxsize=maxsize;
      pixel=(new int[maxsize])-1;
      size=0;
    }

    ~staticheap() {delete[](pixel+1);}

    virtual int order(const int &pix1,const int &pix2) = 0;

    void push(int pix) {
      size++;
      pixel[size]=pix; bckptr[pix]=size;
      upheap(size);
    }

    int pop(int &pix) {
      if (size) {
        pix=pixel[1]; pixel[1]=pixel[size]; bckptr[pixel[1]]=1;
        size--; downheap(1); return 1;
      } else return 0;
    }

    void upheap(int node) {
      int parent, pix=pixel[node];
      while ((parent=(node>>1)) && !order(pixel[parent],pix)) {
        pixel[node]=pixel[parent]; bckptr[pixel[node]]=node; node=parent;
      }
      pixel[node]=pix; bckptr[pix]=node;
    }

    void downheap(int node) {
      int child, pix=pixel[node];
      while ((child=(node<<1))<=size) {
        if (child<size && order(pixel[child+1],pixel[child])) child++;
        if (order(pix,pixel[child])) break;
        pixel[node]=pixel[child]; bckptr[pixel[node]]=node; node=child;
      }
      pixel[node]=pix; bckptr[pix]=node;
    }
};


class minheap : public staticheap {

  public:

    minheap(float *dist,int *bckptr,int maxsize) :
      staticheap(dist,bckptr,maxsize) {;}

    int order(const int &pix1,const int &pix2) {return dist[pix1]<=dist[pix2];}
};


class maxheap : public staticheap {

  public:

    maxheap(float *dist,int *bckptr,int maxsize) :
      staticheap(dist,bckptr,maxsize) {;}

    int order(const int &pix1,const int &pix2) {return dist[pix1]>=dist[pix2];}
};


#define X (1)
#define Y (xsize)

enum {XPOS=1,XNEG=2,XBOTH=3, YPOS=4,YNEG=8,YBOTH=12};
enum {NEW=0, ACTIVE=1, DONE=2};

inline float min(float a,float b) {return a<=b ? a : b;}

float solve(int i,int j,int xsize,int ysize,int *status,float *dist, int *costMap) {
  
  float xnhbr,ynhbr;                //Distance values at neighbors
  float sum,diff;                   //Used in solving quadratic equation
  int nbhd=0;                       //Neighborhood bits
  int p=i+j*xsize;
  float newdist;

  //Determine which neighbors contain usable distance values already

  if (i>0       && status[p-X]!=NEW) nbhd|=XNEG;
  if (i<xsize-1 && status[p+X]!=NEW) nbhd|=XPOS;
  if (j>0       && status[p-Y]!=NEW) nbhd|=YNEG;
  if (j<ysize-1 && status[p+Y]!=NEW) nbhd|=YPOS;

  //Ignore neighbors with larger distances than current point (if applicable)

  if (status[p]!=NEW) {
    if ((nbhd&XNEG) && dist[p-X]>=dist[p]) nbhd-=XNEG;
    if ((nbhd&XPOS) && dist[p+X]>=dist[p]) nbhd-=XPOS;
    if ((nbhd&YNEG) && dist[p-Y]>=dist[p]) nbhd-=YNEG;
    if ((nbhd&YPOS) && dist[p+Y]>=dist[p]) nbhd-=YPOS;
  }

  //Determine which neighbor distance values to use in each direction
  int loc;
  switch (nbhd&XBOTH) {  //Distance to use for x-neighbor
    case XBOTH: 
    	if (dist[p-X] < dist[p+X] ){
    		xnhbr = dist[p-X];
    		loc = p-X;
    	}
		xnhbr = dist[p+X];
		loc = p+X;
    break;
    
    case XNEG:  xnhbr=dist[p-X]; loc = p-X;  break;
    case XPOS:  xnhbr=dist[p+X]; loc = p+X;  break;
  }
  
  switch (nbhd&YBOTH) {  //Distance to use for y-neighbor
  case YBOTH: 
  	if (dist[p-Y] < dist[p+Y] ){
  		ynhbr = dist[p-Y];
  		loc = p-Y;
  	}
		ynhbr = dist[p+Y];
		loc = p+Y;
  break;
    case YNEG:  ynhbr=dist[p-Y]; loc = p-Y;  break;
    case YPOS:  ynhbr=dist[p+Y]; loc = p+Y;  break;
  }

  //Solve for distance at current point based on available neighbor values

  if (!(nbhd&XBOTH))  newdist=ynhbr+ costMap[loc]; //costMap[loc];
  else if (!(nbhd&YBOTH)) {newdist=xnhbr+ costMap[loc]; }
  else {
	  
	if(nbhd==10)      loc = p - X - Y;
	else if(nbhd==9)  loc = p + X - Y; 
	else if(nbhd==5)  loc = p + X + Y;
	else if(nbhd==6)  loc = p - X + Y;
		
    sum=xnhbr+ynhbr; diff=xnhbr-ynhbr; diff*=diff;
    newdist=(sum+sqrt((2.0*(int)costMap[loc])-diff))/2.0;
    if(newdist!=newdist) {
    	if(ynhbr <= xnhbr)
    		newdist = ynhbr+costMap[loc];
    	else
    		newdist = xnhbr+costMap[loc];
    }
  }

  //Make sure new distance is less than old distance (else return old distance)

  if (status[p]!=NEW && dist[p]<newdist) return dist[p];
  else                                   return newdist;
}
 
#define VALID_LOCATION(i,j)                                                    \
  (s->setLocation(i,j), s->isValidLocation(obst,xsize,ysize))

void computeDistances(
 unsigned char *obst,shape *s,int dest,float *dist,int xsize,int ysize, int *costMap
)
//obst = binary input obstacle array (xsize by ysize)
//dist = output array of distances (xsize by ysize)
// s   = shape object to be navigated through the obstacles
//dest = grid index of destination location (where distance = 0)
//xsize,ysize= dimensions of obstacle image
//zsize = (ignored in this 2D version of the code)
{
	
  int p,q;                          //Index of current point and neighbor
  int i,j;                          //2D offsets of current point
  int gridsize=xsize*ysize;         //Total size of grid

  int *status=new int[gridsize];    //Status (NEW/ACTIVE/DONE) of grid points
  int *bckptr=new int[gridsize];    //Backpointers to grid points within heap

  minheap heap(dist,bckptr,gridsize);   //Heap for valid location grid points

  //Initialize status labels

  memset(status,NEW,gridsize*sizeof(int));

  //Initialize heap

  dist[dest]=0;
  heap.push(dest);

  //Build distance function by marching outwards from destination location

  while (heap.pop(p)) {

    status[p]=DONE;
    j=p/xsize; i=p-j*xsize;

    //Update -X neighbor
    if (i>0 && status[q=p-X]!=DONE && VALID_LOCATION(i-1,j)) {
      dist[q]=solve(i-1,j,xsize,ysize,status,dist, costMap);
      if (status[q]==NEW) {status[q]=ACTIVE; heap.push(q);}
      else heap.upheap(bckptr[q]);
    }

    //Update +X neighbor
    if (i<xsize-1 && status[q=p+X]!=DONE && VALID_LOCATION(i+1,j)) {
      dist[q]=solve(i+1,j,xsize,ysize,status,dist, costMap);
      if (status[q]==NEW) {status[q]=ACTIVE; heap.push(q);}
      else heap.upheap(bckptr[q]);
    }

    //Update -Y neighbor
    if (j>0 && status[q=p-Y]!=DONE && VALID_LOCATION(i,j-1)) {
      dist[q]=solve(i,j-1,xsize,ysize,status,dist, costMap);
      if (status[q]==NEW) {status[q]=ACTIVE; heap.push(q);}
      else heap.upheap(bckptr[q]);
    }

    //Update +Y neighbor
    if (j<ysize-1 && status[q=p+Y]!=DONE && VALID_LOCATION(i,j+1)) {
      dist[q]=solve(i,j+1,xsize,ysize,status,dist,costMap);
      if (status[q]==NEW) {status[q]=ACTIVE; heap.push(q);}
      else heap.upheap(bckptr[q]);
    }
  }

  delete[] status; delete[] bckptr;
}

void CreatePotentialfield(int *costMap, int xsize, int ysize, int gridsize, int maxd){
	
	int d=maxd, dnext;
	int x,y,dx,dy;
	for (int i = 0 ; i < gridsize; i++){
		if (costMap[i]==0)  costMap[i]=d;
		else costMap[i]=1;

	}
	
	
	while(d>0){
		for(int i = 0; i < gridsize; i++){
			if (costMap[i]==d){
				// dnext = rint(d/1.1)-1;
				  dnext = d-1;
				 y=i/xsize; 
				 x=i-y*xsize;
				
			     //Update +Y neighbor
				 dy = (i+xsize)/xsize;
				 dx = (i+xsize)-dy*xsize;
			     if ((x==dx) && (i+xsize < gridsize) && (costMap[i+xsize] < d)) costMap[i+xsize]=dnext; 

				 //Update -Y neighbor
				 dy = (i-xsize)/xsize;
				 dx = (i-xsize)-dy*xsize;
			     if ((x==dx) && (i-xsize > 0) && (costMap[i-xsize] < d))  costMap[i-xsize]=dnext;  
				 
				 //Update -X neighbor
				 dy = (i-1)/xsize;
			     if ((y==dy) && (i-1 > 0) && (costMap[i-1] < d)) costMap[i-1]=dnext;  
			     
			     //Update +X neighbor
				 dy = (i+1)/xsize;
			     if ((y==dy) && (i+ 1 < gridsize) && (costMap[i+1] < d))  costMap[i+1]=dnext; 
			}	
		}
		
		d=dnext;
	}
}
