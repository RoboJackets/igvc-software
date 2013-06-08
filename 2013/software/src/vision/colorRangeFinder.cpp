#include "vision/colorRangeFinder.h"

//Reads in an image of a grassy environment with non-grass elements replaced with black
//Returns color ranges for red, green, and blue
using namespace cv;
using namespace std;

void findRange()
{
    //Set up variables
    int rMin, rMax, gMin, gMax, bMin, bMax;
    int rows, cols, r, c;
    int blue,green,red;
    Mat grass;


    rMin = gMin = bMin = 255;
    rMax = gMax = bMax = 0;

    grass = imread("/home/alex/Desktop/grass.png");

    rows = grass.rows;
    cols = grass.cols;

    for(r=0; r<rows; r++)
    {
       for(c=0; c<cols; c++)
       {
           blue = (int)grass.data[grass.step[0]*r + grass.step[1]* c + 0];
           green = (int)grass.data[grass.step[0]*r + grass.step[1]* c + 1];
           red = (int)grass.data[grass.step[0]*r + grass.step[1]* c + 2];

           if ((red >0) && (green>0) && (blue>0))
           {
               bMax = (blue > bMax) ? blue : bMax;
               gMax = (green > gMax) ? green : gMax;
               rMax = (red > rMax) ? red : rMax;

               bMin = (blue < bMin) ? blue : bMin;
               gMin = (green < gMin) ? green : gMin;
               rMin = (red < rMin) ? red : rMin;


           }
        }
    }
    showMatchingPixels(bMin, bMax, gMin, gMax, rMin, rMax);
    cout << "blue min was " << bMin << " and Max was " << bMax << endl;
    cout << "green min was " << gMin << " and Max was " << gMax << endl;
    cout << "red min was " << rMin << " and Max was " << rMax << endl;
}

//takes a picture and determines which

void showMatchingPixels(int bMin, int bMax, int gMin, int gMax, int rMin, int rMax, int reduction)
{
    bMin+=reduction;
    gMin+=reduction;
    rMin+=reduction;
    bMax-=reduction;
    gMax-=reduction;
    rMax-=reduction;
    int r, c, rows, cols;
    uchar *blue, *green, *red;
    string img = "/home/alex/Desktop/BenCode_534_small.jpg";
    Mat track = imread(img);

    rows = track.rows;
    cols = track.cols;

    for(r=0; r<rows; r++)
    {
       for(c=0; c<cols; c++)
       {
         blue = &track.data[track.step[0]*r + track.step[1]* c + 0];
         green = &track.data[track.step[0]*r + track.step[1]* c + 1];
         red = &track.data[track.step[0]*r + track.step[1]* c + 2];
         if (((*blue >= bMin) &&(*blue <= bMax)) && ((*green >= gMin) &&(*green <= gMax)) && ((*red >= rMin) &&(*red <= rMax)))
         {
         }
         else
         {
          *blue=0;
          *green=0;
          *red=0;
         }
       }
    }

    string winName = "window1";
    namedWindow( winName, CV_WINDOW_AUTOSIZE );
    imshow(winName, track);
    char dat=0;
    imwrite("/home/alex/Desktop/BenCode_534_small_green.jpg", track);
    while (dat==0)
    {
      dat = waitKey(50000);
    }

}



void showMatchingPixels(int bMin, int bMax, int gMin, int gMax, int rMin, int rMax, int reduction, string img)
{
    bMin+=reduction;
    gMin+=reduction;
    rMin+=reduction;
    bMax-=reduction;
    gMax-=reduction;
    rMax-=reduction;
    int r, c, rows, cols;
    uchar *blue, *green, *red;

    Mat track = imread(img);

    rows = track.rows;
    cols = track.cols;

    for(r=0; r<rows; r++)
    {
       for(c=0; c<cols; c++)
       {
         blue = &track.data[track.step[0]*r + track.step[1]* c + 0];
         green = &track.data[track.step[0]*r + track.step[1]* c + 1];
         red = &track.data[track.step[0]*r + track.step[1]* c + 2];
         if (((*blue >= bMin) &&(*blue <= bMax)) && ((*green >= gMin) &&(*green <= gMax)) && ((*red >= rMin) &&(*red <= rMax)))
         {
         }
         else
         {
          *blue=0;
          *green=0;
          *red=0;
         }
       }
    }

    string winName = "window1";
    namedWindow( winName, CV_WINDOW_AUTOSIZE );
    imshow(winName, track);
    char dat=0;
    imwrite("/home/alex/Desktop/BenCode_534_small_green.jpg", track);
    while (dat==0)
    {
      dat = waitKey(50000);
    }
}
