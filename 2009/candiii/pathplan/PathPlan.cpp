#include "PathPlan.h"
#include <cstring>


PathPlan::PathPlan()
{
    initialized=0;
}

PathPlan::~PathPlan()
{
    //Deallocate memory
    ;
    delete[] cost;
    delete[] coord;
    delete[] vect;
    delete[] costMap;
    delete[] ptracker;
    delete[] status;
    delete[] bckptr;

}

void PathPlan::Init(int x_size, int y_size,  int widthRobo, int heightRobo)
{
    xsize=x_size;
    ysize=y_size;

    gridsize=xsize*ysize;     //Total number of pixels in the grid
    costMap = new int [gridsize];

    myshape=&myrect;
    myrect.setParams(widthRobo,heightRobo);

    cost=new float[gridsize];               //Allocate array for cost
    coord = new int[1000];
    vect = new int[1000];
    ptracker = new int[1000];

    status=new int[gridsize];    //Status (NEW/ACTIVE/DONE) of grid points
    bckptr=new int[gridsize];    //Backpointers to grid points within heap


    initialized=1;


}

int PathPlan::navigate(unsigned char *image,int xsize, int ysize, int xstart,int ystart,int &xend, int &yend, int scaredD, int widthRobo, int heightRobo)
{

    if (!initialized) Init(xsize,ysize,widthRobo,heightRobo);

    int p1 = xstart+xsize*ystart; //Starting point
    int p2=xend+xsize*yend; //Destination point
    int steps = 0;
    int sizeC = -1;
    int sizeV = -1;
    int sizeP = -1;
    int oldY = p1/Y;
    int oldX = p1-oldY*Y;
    int diffx, diffy;


    for (int p=0; p<gridsize; p++)
    {
        costMap[p]=(int)image[p];
    }

    CreatePotentialfield(costMap, xsize, gridsize,scaredD);



    //Set starting location and draw starting shape outline

    myshape->setLocation(xstart,ystart);

    if (myshape->isValidLocation(image,xsize,ysize))
    {
        myshape->drawOutline(image,xsize);      //Draw starting shape
    }
    else
    {
        cout << "Illegal start location." << endl;
        //exit(1);
        return 0;
    }

    //Set destination location and draw starting shape outline

    myshape->setLocation(xend,yend);

    if (myshape->isValidLocation(image,xsize,ysize))
    {
        myshape->drawOutline(image,xsize);      //Draw destination shape
    }
    else
    {
        cout << "Illegal destination." << endl;
        //exit(1);
        return 0;
    }


    //Compute total cost (distance) function at allowable (x,y,theta) locations

    for (int p=0; p<gridsize; p++)
        cost[p]=gridsize; //Initialize cost values

    computeDistances(image,myshape,p2,cost,xsize,ysize,costMap);

    for (int i = 0; i<1000; i++)
        coord[i]=xstart/2;//default to center

    //Move shape from starting point to destination via the optimal path



    p2=p1;     //Initialize neighbor point p2 to the starting point p1
    int j,i;
    float val;

    do
    {
        p1=p2;                                   //Move p1 to smallest neighbor p2
        j=p1/Y;
        i=p1-j*Y;  //indeces of current point
        ptracker[++sizeP] = p1;
        coord[++sizeC] = i;
        coord[++sizeC] = j;

        //Periodically draw shape (every __ time steps) at the current point p1

        if (steps++ == 20)
        {
            myshape->setLocation(i,j);
            myshape->drawOutline(image,xsize);

            diffx = coord[sizeC-1] -  oldX;
            diffy = (ysize - coord[sizeC]) - (ysize - oldY);

            vect[++sizeV] = (int)sqrt(diffx*diffx + diffy*diffy);
            vect[++sizeV] = (int)((180/PI)*atan2(diffy, diffx));
            oldX = coord[sizeC-1];
            oldY = coord[sizeC];
            steps = 0;
        }

        //Get index of smallest neigbhor p2 to the current point p1
        val=cost[p1];
        if (j>0       && cost[p1-Y]<val)
        {
            val=cost[p1-Y];
            p2=p1-Y;
        }
        if (i>0       && cost[p1-X]<val)
        {
            val=cost[p1-X];
            p2=p1-X;
        }
        if (i<xsize-1 && cost[p1+X]<val)
        {
            val=cost[p1+X];
            p2=p1+X;
        }
        if (j<ysize-1 && cost[p1+Y]<val)
        {
            val=cost[p1+Y];
            p2=p1+Y;
        }


    }
    while (p1!=p2);

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
    int future = 100; // must be even number and less than closenessThresh
    int gx = coord[future];
    int gy = coord[future+1];
    xend = gx;
    yend = gy;




    return 1;
}


void PathPlan::computeDistances(unsigned char *obst,shape *s,int dest,float *dist,int xsize,int ysize, int *costMap)
{

    //obst = binary input obstacle array (xsize by ysize)
    //dist = output array of distances (xsize by ysize)
    // s   = shape object to be navigated through the obstacles
    //dest = grid index of destination location (where distance = 0)
    //xsize,ysize= dimensions of obstacle image
    //zsize = (ignored in this 2D version of the code)

    int p,q;                          //Index of current point and neighbor
    int i,j;                          //2D offsets of current point



    minheap heap(dist,bckptr,gridsize);   //Heap for valid location grid points

    //Initialize status labels

    memset(status,NEW,gridsize*sizeof(int));

    //Initialize heap

    dist[dest]=0;
    heap.push(dest);

    //Build distance function by marching outwards from destination location

    while (heap.pop(p))
    {

        status[p]=DONE;
        j=p/xsize;
        i=p-j*xsize;

        //Update -X neighbor
        if (i>0 && status[q=p-X]!=DONE && VALID_LOCATION(i-1,j))
        {
            dist[q]=solve(i-1,j,xsize,ysize,status,dist, costMap);
            if (status[q]==NEW)
            {
                status[q]=ACTIVE;
                heap.push(q);
            }
            else
                heap.upheap(bckptr[q]);
        }

        //Update +X neighbor
        if (i<xsize-1 && status[q=p+X]!=DONE && VALID_LOCATION(i+1,j))
        {
            dist[q]=solve(i+1,j,xsize,ysize,status,dist, costMap);
            if (status[q]==NEW)
            {
                status[q]=ACTIVE;
                heap.push(q);
            }
            else
                heap.upheap(bckptr[q]);
        }

        //Update -Y neighbor
        if (j>0 && status[q=p-Y]!=DONE && VALID_LOCATION(i,j-1))
        {
            dist[q]=solve(i,j-1,xsize,ysize,status,dist, costMap);
            if (status[q]==NEW)
            {
                status[q]=ACTIVE;
                heap.push(q);
            }
            else
                heap.upheap(bckptr[q]);
        }

        //Update +Y neighbor
        if (j<ysize-1 && status[q=p+Y]!=DONE && VALID_LOCATION(i,j+1))
        {
            dist[q]=solve(i,j+1,xsize,ysize,status,dist,costMap);
            if (status[q]==NEW)
            {
                status[q]=ACTIVE;
                heap.push(q);
            }
            else
                heap.upheap(bckptr[q]);
        }
    }


}

void PathPlan::CreatePotentialfield(int *costMap, int xsize,  int gridsize, int maxd)
{

    int d=maxd;
    int dnext;
    int x,y,dx,dy;
    for (int i = 0 ; i < gridsize; i++)
    {
        if (costMap[i]==0)
            costMap[i]=d;
        else
            costMap[i]=1;

    }

    while (d>0)
    {
        for (int i = 0; i < gridsize; i++)
        {
            dnext = d-1;
            if (costMap[i]==d)
            {


                y=i/xsize;
                x=i-y*xsize;

                //Update +Y neighbor
                dy = (i+xsize)/xsize;
                dx = (i+xsize)-dy*xsize;
                if ((x==dx) && (i+xsize < gridsize) && (costMap[i+xsize] < d))
                    costMap[i+xsize]=dnext;

                //Update -Y neighbor
                dy = (i-xsize)/xsize;
                dx = (i-xsize)-dy*xsize;
                if ((x==dx) && (i-xsize > 0) && (costMap[i-xsize] < d))
                    costMap[i-xsize]=dnext;

                //Update -X neighbor
                dy = (i-1)/xsize;
                if ((y==dy) && (i-1 > 0) && (costMap[i-1] < d))
                    costMap[i-1]=dnext;

                //Update +X neighbor
                dy = (i+1)/xsize;
                if ((y==dy) && (i+ 1 < gridsize) && (costMap[i+1] < d))
                    costMap[i+1]=dnext;
            }
        }

        d=dnext;
    }
}

float PathPlan::solve(int i,int j,int xsize,int ysize,int *status,float *dist, int *costMap)
{

    float xnhbr,ynhbr;                //Distance values at neighbors
    float sum,diff;                   //Used in solving quadratic equation
    int nbhd=0;                       //Neighborhood bits
    int p=i+j*xsize;
    float newdist;
    int loc;

    //Determine which neighbors contain usable distance values already

    if (i>0       && status[p-X]!=NEW)
        nbhd|=XNEG;
    if (i<xsize-1 && status[p+X]!=NEW)
        nbhd|=XPOS;
    if (j>0       && status[p-Y]!=NEW)
        nbhd|=YNEG;
    if (j<ysize-1 && status[p+Y]!=NEW)
        nbhd|=YPOS;

    //Ignore neighbors with larger distances than current point (if applicable)

    if (status[p]!=NEW)
    {
        if ((nbhd&XNEG) && dist[p-X]>=dist[p])
            nbhd-=XNEG;
        if ((nbhd&XPOS) && dist[p+X]>=dist[p])
            nbhd-=XPOS;
        if ((nbhd&YNEG) && dist[p-Y]>=dist[p])
            nbhd-=YNEG;
        if ((nbhd&YPOS) && dist[p+Y]>=dist[p])
            nbhd-=YPOS;
    }

    //Determine which neighbor distance values to use in each direction

    switch (nbhd&XBOTH)    //Distance to use for x-neighbor
    {
    case XBOTH:
        if (dist[p-X] < dist[p+X] )
        {
            xnhbr = dist[p-X];
            loc = p-X;
        }
        xnhbr = dist[p+X];
        loc = p+X;
        break;

    case XNEG:
        xnhbr=dist[p-X];
        loc = p-X;
        break;
    case XPOS:
        xnhbr=dist[p+X];
        loc = p+X;
        break;
    }

    switch (nbhd&YBOTH)    //Distance to use for y-neighbor
    {
    case YBOTH:
        if (dist[p-Y] < dist[p+Y] )
        {
            ynhbr = dist[p-Y];
            loc = p-Y;
        }
        ynhbr = dist[p+Y];
        loc = p+Y;
        break;
    case YNEG:
        ynhbr=dist[p-Y];
        loc = p-Y;
        break;
    case YPOS:
        ynhbr=dist[p+Y];
        loc = p+Y;
        break;
    }

    //Solve for distance at current point based on available neighbor values

    if (!(nbhd&XBOTH))
        newdist=ynhbr+ costMap[loc]; //costMap[loc];
    else if (!(nbhd&YBOTH))
    {
        newdist=xnhbr+ costMap[loc];
    }
    else
    {

        if (nbhd==10)
            loc = p - X - Y;
        else if (nbhd==9)
            loc = p + X - Y;
        else if (nbhd==5)
            loc = p + X + Y;
        else if (nbhd==6)
            loc = p - X + Y;

        sum=xnhbr+ynhbr;
        diff=xnhbr-ynhbr;
        diff*=diff;
        newdist=(sum+sqrt((2.0*(int)costMap[loc])-diff))/2.0;
        if (newdist!=newdist)
        {
            if (ynhbr <= xnhbr)
                newdist = ynhbr+costMap[loc];
            else
                newdist = xnhbr+costMap[loc];
        }
    }

    //Make sure new distance is less than old distance (else return old distance)

    if (status[p]!=NEW && dist[p]<newdist)
        return dist[p];
    else
        return newdist;
}


