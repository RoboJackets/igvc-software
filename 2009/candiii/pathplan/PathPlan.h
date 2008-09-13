#ifndef PATHPLAN_H_
#define PATHPLAN_H_

/*
 * This file contains the path planning code ported from the 2008 pathplan files,
 *   and put into objects/classes.
 *
 * TODO: THIS FILE NEEDS TO BE OPTIMIZED !!!
 *
 *   by: Chris McClanahan
 *
 */


/* copied from distance2D.h */
#include <stdlib.h>      //Included for function exit()
#include <fstream>       //Included for ifstream class
#include <iostream>      //Included for ostream class (cout)
#include "shape.h"
#define X (1)
#define Y (xsize)
#define VALID_LOCATION(i,j)  (s->setLocation(i,j), s->isValidLocation(obst,xsize,ysize))
enum {XPOS=1,XNEG=2,XBOTH=3, YPOS=4,YNEG=8,YBOTH=12};
enum {NEW=0, ACTIVE=1, DONE=2};



class PathPlan {
public:
    PathPlan();
    virtual ~PathPlan();
    
    void Init(int x_size, int y_size,  int widthRobo, int heightRobo);
    /* internal variables to speed things up (no re-creations in each function) */
    int* costMap;
    int gridsize;
    int xsize;
    int ysize;
    rectangle myrect;
    shape *myshape;  //Base pointer to one of the above 3 derived shape objects    
    float *cost;
    int *coord;
    int *vect;
    int *ptracker;
    int *status;    //Status (NEW/ACTIVE/DONE) of grid points
    int *bckptr;    //Backpointers to grid points within heap
    int initialized;


    /* copied from pathplan.h */

    inline float rint(float x) {
        return float((int)(x+.5));
    }

    void computeDistances(unsigned char *obst, shape *s,int dest,float *dist,int xsize,int ysize, int *costMap);

    //void CreatePotentialfield(int *costMap, int xsize, int ysize, int gridsize, int maxd);
    void CreatePotentialfield(int *costMap, int xsize, int gridsize, int maxd);

    int navigate(unsigned char *image,int xsize, int ysize, int xstart,int ystart,int &xend, int &yend, int scaredD, int widthRobo, int heightRobo);
    //int navigate(unsigned char *image, int xstart,int ystart, int &xend, int &yend,  int scaredD);

    /* copied from distance2D.h */

    inline float min(float a,float b) {
        return a<=b ? a : b;
    }

    float solve(int i,int j,int xsize,int ysize,int *status,float *dist, int *costMap);

};



/* copied from distance2D.h */

class staticheap {

protected:
    int *pixel;  //Pointer to array of pixels stored in heap (1-offset)
    int size;    //Current size of heap
    int maxsize; //Amount of space allocated to pixel and size arrays
    float *dist; //Signed distance array
    int *bckptr; //Array of backpointers

public:
    staticheap(float *dist,int *bckptr,int maxsize) {
        if (maxsize<0)
            maxsize=0;
        this->dist=dist;
        this->bckptr=bckptr;
        this->maxsize=maxsize;
        pixel=(new int[maxsize])-1;
        size=0;
    }

    ~staticheap() {
        delete[](pixel+1);
    }

    virtual int order(const int &pix1,const int &pix2) = 0;

    void push(int pix) {
        size++;
        pixel[size]=pix;
        bckptr[pix]=size;
        upheap(size);
    }

    int pop(int &pix) {
        if (size) {
            pix=pixel[1];
            pixel[1]=pixel[size];
            bckptr[pixel[1]]=1;
            size--;
            downheap(1);
            return 1;
        } else
            return 0;
    }

    void upheap(int node) {
        int parent, pix=pixel[node];
        while ((parent=(node>>1)) && !order(pixel[parent],pix)) {
            pixel[node]=pixel[parent];
            bckptr[pixel[node]]=node;
            node=parent;
        }
        pixel[node]=pix;
        bckptr[pix]=node;
    }

    void downheap(int node) {
        int child, pix=pixel[node];
        while ((child=(node<<1))<=size) {
            if (child<size && order(pixel[child+1],pixel[child]))
                child++;
            if (order(pix,pixel[child]))
                break;
            pixel[node]=pixel[child];
            bckptr[pixel[node]]=node;
            node=child;
        }
        pixel[node]=pix;
        bckptr[pix]=node;
    }
};


class minheap : public staticheap {
public:
    minheap(float *dist,int *bckptr,int maxsize) :
            staticheap(dist,bckptr,maxsize) {
        ;
    }
    int order(const int &pix1,const int &pix2) {
        return dist[pix1]<=dist[pix2];
    }
};


class maxheap : public staticheap {
public:
    maxheap(float *dist,int *bckptr,int maxsize) :
            staticheap(dist,bckptr,maxsize) {
        ;
    }
    int order(const int &pix1,const int &pix2) {
        return dist[pix1]>=dist[pix2];
    }
};




#endif /*PATHPLAN_H_*/
