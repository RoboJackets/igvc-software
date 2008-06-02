#include <stdlib.h>      //Included for function exit()
#include <fstream>       //Included for ifstream class
#include <iostream>      //Included for ostream class (cout)
#include "shape.h"

using namespace std;

inline float rint(float x) {return float((int)(x+.5));}

//Delcaration of functions to read/write pgm images
//-------------------------------------------------

int writePGM(const char *filename,unsigned char *image,int xsize,int ysize);
int writeTxt(const char *filename,int *array,int size);
int writeCost(const char *filename,int *image,int xsize,int ysize);
int writeDist(const char *filename,float *image,int xsize,int ysize);

//Declaration of function to compute total costs to destination shape in 3D grid
//----------------------------------------------------------------------------
//obst = binary input obstacle array (xsize by ysize)
//cost = output array of total costs (xsize by ysize by zsize)
// s   = shape object to be navigated through the obstacles
//dest = grid index of destination location (where cost = 0)

void computeDistances(
 unsigned char *obst,shape *s,int dest,float *dist,int xsize,int ysize, int *costMap
);

void CreatePotentialfield(int *costMap, int xsize, int ysize, int gridsize, int maxd);

int navigate(unsigned char *image,int xsize, int ysize, int xstart,int ystart,int &xend, int &yend, int scaredD, int widthRobo, int heightRobo){
		
		
  int *costMap;
  int gridsize=xsize*ysize;     //Total number of pixels in the grid
  costMap = new int [gridsize];
  
  for (int p=0; p<gridsize; p++) {costMap[p]=(int)image[p];}
  
  CreatePotentialfield(costMap, xsize, ysize, gridsize,scaredD);

  //Shape Rect  
  rectangle myrect;
  shape *myshape;  //Base pointer to one of the above 3 derived shape objects
  myshape=&myrect; 
  myrect.setParams(widthRobo,heightRobo);
 
  //Set starting location and draw starting shape outline
  int p1 = xstart+xsize*ystart; //Starting point
  myshape->setLocation(xstart,ystart);
  
  if (myshape->isValidLocation(image,xsize,ysize)) {
    myshape->drawOutline(image,xsize,ysize);      //Draw starting shape
  } else {
    cout << "Illegal start location." << endl;
    //exit(1);
    return 0;
  }

  //Set destination location and draw starting shape outline
  int p2=xend+xsize*yend; //Destination point
  myshape->setLocation(xend,yend);
  
  if (myshape->isValidLocation(image,xsize,ysize)) {
    myshape->drawOutline(image,xsize,ysize);      //Draw destination shape
  } else {
    cout << "Illegal destination." << endl;
    //exit(1);
    return 0;
  }
  
  int startP = p1;
  int endP = p2;

  //Compute total cost (distance) function at allowable (x,y,theta) locations

  float *cost=new float[gridsize];               //Allocate array for cost
  for (int p=0; p<gridsize; p++) cost[p]=gridsize; //Initialize cost values

  computeDistances(image,myshape,p2,cost,xsize,ysize,costMap);


  //Move shape from starting point to destination via the optimal path

  int steps=0;
  int sizeC = -1;
  int sizeV = -1;
  int sizeP = -1;
  int *coord = new int[1000];
  for (int i = 0; i<1000; i++) coord[i]=xstart/2;//default to center
  int *vect = new int[1000];
  int *ptracker = new int[1000];
  
  int X=1, Y=xsize;   //Offsets needed to move by one pixel
  int oldY = p1/Y;
  int oldX = p1-oldY*Y;
  int diffx, diffy;
  
  p2=p1;     //Initialize neighbor point p2 to the starting point p1

  do {
    p1=p2;                                   //Move p1 to smallest neighbor p2
    int j=p1/Y, i=p1-j*Y;  //indeces of current point
    ptracker[++sizeP] = p1;
    coord[++sizeC] = i;
    coord[++sizeC] = j;
    
    //Periodically draw shape (every 15 time steps) at the current point p1

    if (steps++ == 10) {
      myshape->setLocation(i,j);
      myshape->drawOutline(image,xsize,ysize);
      
      diffx = coord[sizeC-1] -  oldX;
      diffy = (ysize - coord[sizeC]) - (ysize - oldY);

      vect[++sizeV] = (int)sqrt(diffx*diffx + diffy*diffy);
      vect[++sizeV] = (int)((180/PI)*atan2(diffy, diffx));
      oldX = coord[sizeC-1];
      oldY = coord[sizeC];
      steps = 0;
    }

    //Get index of smallest neigbhor p2 to the current point p1
    float val=cost[p1];
    if (j>0       && cost[p1-Y]<val) {val=cost[p1-Y]; p2=p1-Y;}
    if (i>0       && cost[p1-X]<val) {val=cost[p1-X]; p2=p1-X;}
    if (i<xsize-1 && cost[p1+X]<val) {val=cost[p1+X]; p2=p1+X;}
    if (j<ysize-1 && cost[p1+Y]<val) {val=cost[p1+Y]; p2=p1+Y;}

 
  } while (p1!=p2);
/*
  //Write output image
  if (!writePGM("output.pgm",image,xsize,ysize))
    cout << "Unable to write output image: " << endl;
  //Write coordinates text file
  if (!writeTxt("coordinates.txt",coord,sizeC))
    cout << "Unable to write coord txt file: " << endl;
  //Write vectors text file
  if (!writeTxt("vectors.txt",vect,sizeV))
    cout << "Unable to write vect txt file: " << endl;
  //Write test text file
  costMap[startP] = 100000;
  costMap[endP] = 100000;
  for(int i = 0; i < sizeP; i++){
	  costMap[ptracker[i]]=0;
  }
  if (!writeCost("costMap.txt",costMap,xsize,ysize))
    cout << "Unable to write test file: " << endl;
  //Write dist text file
  cost[startP] = 100000;
  cost[endP] = 100000;
  if (!writeDist("dist.txt",cost,xsize,ysize))
    cout << "Unable to write test dist file: " << endl;
*/

  // Update input goal to new goal
  // use a coordinate about 60 steps out
  int future =60; // must be even number and less than closenessThresh
  int gx = coord[future];
  int gy = coord[future+1];
  xend = gx;
  yend = gy;


  //Deallocate memory

  //delete[] image;
  delete[] cost;
  delete[] coord;
  delete[] vect;
  delete[] costMap;
  delete[] ptracker;

  return 1;
}
