#include "vision.h"
#include <stdio.h>
#include "image_buffers.h"



/*
 * This file contains the robot's primary vision processing code main function.
 *   by: Chris McClanahan
 *
 */

// for scanning the path image /////////////////////////////////////////////////////////////
#define BAD_PIXEL 0		// value to set path image to
#define GOOD_PIXEL 255	// value to set path image to
#define L_R_OFFSET 20 	// pixel spacing from center scan line up
//#define ROBOT_WIDTH 30 	// pixels wide
#define PIXEL_SKIP 2 	// noise filtering threshold in checkPixel()
#define EDGE_PAD 4      // top/bottom padding


Vision::Vision()
{
    init();
}
Vision::~Vision()
{

}


void Vision::init()
{

    /* load in vision settings */
    LoadVisionXMLSettings();


    // font
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 2);

    // display roi for adaptive coloring
    cvNamedWindow("roi",1);


}

/*
 * This function scans from the bottom center of image upward,
 *	checking to see if the width of the robot can progress
 *	any further up the image, while sliding left/right as needed
 */
void Vision::robotWidthScan(IplImage* img, int& goalx, int& goaly)
{

    int center = img->width/2;
    int startx = center - ROBOT_WIDTH/2;
    int endx = startx + ROBOT_WIDTH;
    int y = img->height-EDGE_PAD;
    int x = startx;
    int half = ROBOT_WIDTH/2;
    int i;

    // return -1 on failure
    goalx=-1;
    goaly=-1;

    /* 	scan from bottom center of image upward,
    	checking to see if the width of the robot can progress
    	any further up the image, sliding left/right as needed
    */
    for ( ; y >= EDGE_PAD ; y-- )
    {

        //check left, move right
        for (x=startx; x<img->width-ROBOT_WIDTH-1; x++)
        {
            if (!checkPixel(img,x,y)|| !checkPixel(img,x+half,y) )
            {
                x++;	//slide right
                startx=x;
                endx=x+ROBOT_WIDTH;
            }
            else
            {
                startx=x;
                endx=x+ROBOT_WIDTH;
                break;
            }
        }

        for ( i = startx; i<endx; i++)   //scan along width to not cross over boundary
        {
            if (!checkPixel(img,i,y))
            {
                break;
            }
            else
            {
                goalx = i-half;
                goaly = y;
            }
        }
        if (i>=endx)
        {
            //success, keep going up
            continue;
        }

        //check right, move left
        for (x=endx; x>0+ROBOT_WIDTH; x--)
        {
            if (!checkPixel(img,x,y) || !checkPixel(img, x-half, y) )
            {
                x--;	//slide left
                startx=x-ROBOT_WIDTH;
                endx=x;
            }
            else
            {
                startx=x-ROBOT_WIDTH;
                endx=x;
                break;
            }
        }
        for ( i = endx; i>startx; i--)   //scan along width to not cross over boundary
        {
            if (!checkPixel(img,i,y))
            {
                break;
            }
            else
            {
                goalx = i+half;
                goaly = y;
            }
        }
        if (i<=startx)
        {
            //success, keep going up
            continue;
        }
        else
        {
            break;		//we dont fit left or right
        }
    }

    //sanity check goal
    if (goalx==-1||goaly==-1)
    {
        //not good
    }
    else
    {
        //found goal
        if (!checkPixel(img,goalx,goaly))
        {
            goalx=goaly=-1;	//not good, error in scanning
        }
        else
        {

            /* GOAL! */
            //=== debug : show goal ===//
            //=== visCvPath=320x240 ===//
            cvLine(visCvDebug, cvPoint(center*2,img->height*2-2), cvPoint(goalx*2,goaly*2)       , (CV_RGB(0,0,0)), 2, 8, 0);
            cvLine(visCvDebug, cvPoint((goalx-half)*2,goaly*2)  , cvPoint((goalx+half)*2,goaly*2), (CV_RGB(0,0,0)), 2, 8, 0);
            //=========================//

        }
    }

    // IMPLICITLY:
    // 	return goalx,goaly,
    // 	or set -1 on error

}

/*
 * This function generates the visCvPath image by scanning the
 *   visCvThresh image from the bottom center up, while scanning/filling left and right.
 * As soon as the center pixel is bad, all the above pixels are bad too.
 */
void Vision::visGenPath(IplImage* img)
{
    int width = img->width;
    int height = img->height-EDGE_PAD; // skip noise at bottom
    int goodFirst = 1;
    int x = width/2;
    int blackout = 0;

    // find best column to scan up in
    x = findBestX(img, height, x);

    // scan bottom to top in rows; white = path; black = bad
    // skip the bottom few pixels, due to noise
    for (int y = height; y >= EDGE_PAD ; y--)
    {
        if (checkPixel(img,x,y))  	//check starting point in middle
        {
            goodFirst=1;
        }
        else
        {
            goodFirst=0;
            blackout=1;
        }

        //scan left then right & generate visCvPath image
        scanFillLeft  (visCvPath, x, y, goodFirst, 0      , blackout);
        scanFillRight (visCvPath, x, y, goodFirst, width-1, blackout);

    }//y

    // debug draw black line to see which column was chosen (left,center,right)
    //cvLine(visCvThresh, cvPoint(x,1), cvPoint(x,height-2), (CV_RGB(0,0,0)), 2, 8, 0);

}//visGenPath

/*
 * This function scans up the center (and left/right of center) of thresh image,
 *   and returns the column that went the 'hightest' in the image
 */
int Vision::findBestX(IplImage* img, int height, int center)
{
    int left  = center - L_R_OFFSET;
    int right = center + L_R_OFFSET;
    int heightL = 0;
    int heightR = 0;
    int heightC = 0;
    int y = height;

    // break as soon as we see an obstacle
    for (y = height; y >= EDGE_PAD ; y--)
    {
        if (checkPixel(img,left,y))
            heightL++;
        else
            break;
    }
    for (y = height; y >= EDGE_PAD ; y--)
    {
        if (checkPixel(img,center,y))
            heightC++;
        else
            break;
    }
    for (y = height; y >= EDGE_PAD ; y--)
    {
        if (checkPixel(img,right,y))
            heightR++;
        else
            break;
    }

    if ( heightL > heightC && heightL > heightR )
        return left;
    else if ( heightR > heightC && heightR > heightL )
        return right;
    else
        return center;    // default to center

}

/*
 * This function retuns T/F based on:
 * 		white = good/path
 * 		black = bad/obstacle
 */
int Vision::checkPixel(IplImage* img, int x, int y)
{
    int good = 0;
    //int index = y*img->width+x;

    unsigned char val = img->imageData[y*img->width+x];

    // check: black = bad
    if ( !val )
    {
        // check for noise
        if ( !(img->imageData[ (y-PIXEL_SKIP)*img->width+x ]) )
            good = 0;
        else
            good = 1; // just a small spot, keep going up
    }
    // white = good
    else
    {
        good = 1;

    }

    return good;
}

/*
 *  This function scans from the center of the visCvThresh image to end
 *  	checking for good / bad pixels, setting the visCvPath image accordingly
 */
void Vision::scanFillLeft(IplImage* img, int middleX, int y, int goodFirst, int end, int blackout)
{
    int x=middleX;
    int good;
    int index;
    int offset = y*img->width;

    if (blackout)
    {
        for (;x>=end;x--)  	//fill black
        {
            //set bad
            index = offset+x;
            img->imageData[index] = BAD_PIXEL;
        }
        return;
    }

    if (goodFirst)  		//starting pixel is good
    {
        good=1;
        for (;x>=end;x--)  	//scan left and check
        {
            if (good)
            {
                if (checkPixel(visCvThresh,x,y))
                {
                    //set good
                    index = offset+x;
                    img->imageData[index] = GOOD_PIXEL;
                }
                else
                {
                    //set bad
                    good=0;
                    index = offset+x;
                    img->imageData[index] = BAD_PIXEL;
                }
            }
            else  		//all the rest are bad
            {
                //set bad
                index = offset+x;
                img->imageData[index] = BAD_PIXEL;
            }
        }
    }
    else  		//starting pixel is bad
    {
        good=2;
        for (;x>=end;x--)  	//scan left and check
        {
            if (good==2)  	//in bad spot, check for good spot
            {
                if (checkPixel(visCvThresh,x,y))
                {
                    //set good
                    good=1;
                    index = offset+x;
                    img->imageData[index] = GOOD_PIXEL;
                }
                else
                {
                    //set bad
                    index = offset+x;
                    img->imageData[index] = BAD_PIXEL;
                }
            }
            else if (good==1)  	//a good spot appeared, check for bad again
            {
                if (checkPixel(visCvThresh,x,y))
                {
                    //set good
                    index = offset+x;
                    img->imageData[index] = GOOD_PIXEL;
                }
                else
                {
                    //set bad
                    good=0;
                    index = offset+x;
                    img->imageData[index] = BAD_PIXEL;
                }
            }
            else   //all the rest are bad
            {
                //set bad
                index = offset+x;
                img->imageData[index] = BAD_PIXEL;
            }
        }
    }
}

/*
 *  This function scans from the center of the visCvThresh image to end
 *  	checking for good / bad pixels, setting the visCvPath image accordingly
 */
void Vision::scanFillRight(IplImage* img, int middleX, int y, int goodFirst, int end, int blackout)
{
    int x=middleX;
    int good;
    int index;
    int offset = y*img->width;

    if (blackout)
    {
        for (;x<end;x++)  	//fill black
        {
            //set bad
            index = offset+x;
            img->imageData[index] = BAD_PIXEL;
        }
        return;
    }

    if (goodFirst)  		//starting pixel is good
    {
        good=1;
        for (;x<end;x++)  	//scan right and check
        {
            if (good)
            {
                if (checkPixel(visCvThresh,x,y))
                {
                    //set good
                    index = offset+x;
                    img->imageData[index] = GOOD_PIXEL;
                }
                else
                {
                    //set bad
                    good=0;
                    index = offset+x;
                    img->imageData[index] = BAD_PIXEL;
                }
            }
            else  		//all the rest are bad
            {
                //set bad
                index = offset+x;
                img->imageData[index] = BAD_PIXEL;
            }
        }
    }
    else  		//starting pixel is bad
    {
        good=2;
        for (;x<end;x++)  	//scan right and check
        {
            if (good==2)  	//in bad spot, check for good spot
            {
                if (checkPixel(visCvThresh,x,y))
                {
                    //set good
                    good=1;
                    index = offset+x;
                    img->imageData[index] = GOOD_PIXEL;
                }
                else
                {
                    //set bad
                    index = offset+x;
                    img->imageData[index] = BAD_PIXEL;
                }
            }
            else if (good==1)  	//a good spot appeared, check for bad again
            {
                if (checkPixel(visCvThresh,x,y))
                {
                    //set good
                    index = offset+x;
                    img->imageData[index] = GOOD_PIXEL;
                }
                else
                {
                    //set bad
                    good=0;
                    index = offset+x;
                    img->imageData[index] = BAD_PIXEL;
                }
            }
            else   //all the rest are bad
            {
                //set bad
                index = offset+x;
                img->imageData[index] = BAD_PIXEL;
            }
        }
    }
}

/*
 * Manually check each pixel for special cases not accounted for via HSV
 *
 */
void Vision::preProcessColors(IplImage* img)
{
    unsigned char red,green,blue;

    // pixel data loop
    for (int i = 0; i < img->imageSize-3; i+=3)
    {

        // get pixel data
        blue  = img->imageData[i  ];
        green = img->imageData[i+1];
        red   = img->imageData[i+2];

        // check for neon green
        if ( (green>205) && (green>red) && (green>blue) && (red>blue) )  //&& (blue<65) ){
        {
            // make orange
            img->imageData[i  ] = 0;
            img->imageData[i+1] = 128;
            img->imageData[i+2] = 255;
        }

    }// end loop

}




// image display control
void Vision::ConvertAllImageViews(int trackbarVal)
{

    switch (trackbarVal)
    {
    case 0:
        cvPutText(visCvRaw, "Raw", cvPoint(5,visCvRaw->height-10), &font, CV_RGB(255,255,255));
        cvShowImage("display", visCvRaw);
        break;
    case 1:
        cvPutText(visCvDebug, "Debug", cvPoint(5,visCvRaw->height-10), &font, CV_RGB(255,255,255));
        cvShowImage("display", visCvDebug);
        break;
    case 2:
        cvPutText(visCvPath, "Path", cvPoint(5,visCvPath->height-10), &font, CV_RGB(255,255,255));
        cvShowImage("display", visCvPath);
        break;
    case 3:
        cvPutText(visCvThresh, "Thresh", cvPoint(5,visCvThresh->height-10), &font, CV_RGB(255,255,255));
        cvShowImage("display", visCvThresh);
        break;
    case 4:
        cvPutText(visCvAdapt, "Adaptive", cvPoint(5,visCvRaw->height-10), &font, CV_RGB(255,255,255));
        cvShowImage("display", visCvAdapt);
        break;
    case 5:
        cvPutText(visCvSaturation, "Sat", cvPoint(5,visCvSaturation->height-10), &font, CV_RGB(0,0,0));
        cvShowImage("display", visCvSaturation);
        break;
    case 6:
        cvPutText(visCvHue, "Hue", cvPoint(5,visCvHue->height-10), &font, CV_RGB(0,0,0));
        cvShowImage("display", visCvHue);
        break;
    case 7:
        cvPutText(visCvHSV, "HSV", cvPoint(5,visCvRaw->height-10), &font, CV_RGB(255,255,255));
        cvShowImage("display", visCvHSV);
        break;
        /* future use */
//  case 8:
//  	cvShowImage("display", );
//  	break;

    default:
        break;

    }
    cvWaitKey(10);

}

void Vision::LoadVisionXMLSettings()
{

            {
                satThreshold = 60;
                hueThreshold = 20;
                DO_TRANSFORM = 1;
                ROBOT_WIDTH  = 30;
                adapt_maxDiff= 65;
                adapt_boxPad = 80;
                DO_ADAPTIVE  = 1;
            }


        {
            printf("Vision settings loaded \n");
        }
        printf("thresholds: sat %d  hue %d \n",satThreshold,hueThreshold);
        printf("thresholds: maxDiff %d  boxPad %d \n",adapt_maxDiff,adapt_boxPad);
 

}


void Vision::ThresholdImage(IplImage *src, IplImage *dst, int thresh)
{
    // binary threshold
    cvThreshold(src,dst,thresh,255,CV_THRESH_BINARY_INV);
}

/*
 * Normalizes a 0-255 grayscale image using its min/max values
 */
void Vision::Normalize(IplImage* img)
{
    double min,max,scale,shift;
    cvMinMaxLoc(img, &min, &max, NULL, NULL, NULL);
    //scale = 255.0/(max-min);
    if (max != min)
    {
        scale = 255.0/(max-min);
        shift = 255*(-min)/(max-min);
    }
    else
    {
        scale = 1.0;
        shift = -max;
    }
    cvScale( img, img, scale, shift ); //Normalizes matrix to 0-255 (grayscale)
}

/*
 * Setup a ROI image region to calculate the average red, green, blue values in that area.
 * Use these averages to compare every pixel to check if it is 'close' to that color.
 * Then generate the visCvAdapt image that will turn into visCvPath image.
 */
void Vision::Adapt()
{

    /* define roi corners (upper-left and lower-right) */
    CvPoint UL;
    CvPoint LR;
    if (DO_TRANSFORM)
    {
        UL = cvPoint(  visCvDebug->width/3+adapt_boxPad, visCvDebug->height-adapt_boxPad/3);
        LR = cvPoint(2*visCvDebug->width/3-adapt_boxPad, visCvDebug->height-adapt_boxPad/5);
    }
    else
    {
        UL = cvPoint(  visCvDebug->width/3+adapt_boxPad, visCvDebug->height-adapt_boxPad);
        LR = cvPoint(2*visCvDebug->width/3-adapt_boxPad, visCvDebug->height-adapt_boxPad/2);
    }

    /* setup roi */
    CvRect roi;
    roi.x = UL.x;
    roi.y = UL.y;
    roi.width  = LR.x-UL.x;
    roi.height = LR.y-UL.y;

    /* create and set roi img */
    IplImage* roi_img = cvCreateImage( cvSize(roi.width, roi.height), IPL_DEPTH_8U, 3 );
    cvSetImageROI(visCvDebug,roi);
    cvCopy(visCvDebug,roi_img);
    cvResetImageROI(visCvDebug);

    /* display roi box in separate window */
    cvShowImage( "roi" , roi_img );

    /* get average rgb values in roi */
    int blue=0,green=0,red=0;
    unsigned char ab,ag,ar;
    //for (int i=0; i<roi_img->imageSize-3; i+=3)
    for (int i=roi_img->imageSize-3; i>0; i-=3)
    {
        ab = roi_img->imageData[i  ];
        ag = roi_img->imageData[i+1];
        ar = roi_img->imageData[i+2];
        blue += ab;
        green+= ag;
        red  += ar;
    }
    int n = roi_img->imageSize / 3;
    blue  /= n;
    green /= n;
    red   /= n;

    /* generate visCvAdapt image here!
     *  white=good ~ black=bad */
    //for (int i=0; i<visCvDebug->imageSize-3; i+=3)
    for (int i=visCvDebug->imageSize-3; i>0; i-=3)
    {
        ab = visCvDebug->imageData[i  ];
        ag = visCvDebug->imageData[i+1];
        ar = visCvDebug->imageData[i+2];

        if (    (abs(ab-(unsigned char)blue )<adapt_maxDiff) &&
                (abs(ag-(unsigned char)green)<adapt_maxDiff) &&
                (abs(ar-(unsigned char)red  )<adapt_maxDiff))
        {
            visCvAdapt->imageData[i/3] = GOOD_PIXEL;
            
            /*********************************************************************************/
    	    visCvRaw->imageData[i  ] = 0 ;
    	    visCvRaw->imageData[i+1] = 0 ;
	        visCvRaw->imageData[i+2] = 0 ;            
            /*********************************************************************************/            
            
        }
        else
        {
            visCvAdapt->imageData[i/3] = BAD_PIXEL;         
            
            /*********************************************************************************/
    	    //visCvRaw->imageData[i  ] = 255 ;
    	    //visCvRaw->imageData[i+1] = 255 ;
	        //visCvRaw->imageData[i+2] = 255 ;            
            /*********************************************************************************/     
                                      
        }
    }

    /* display roi box in the image we're using */
    cvRectangle( visCvDebug, UL, LR, CV_RGB(100,0,0), 2, 8, 0);

    /* no memory leaks! */
    cvReleaseImage(&roi_img);

}

/*
 * Performs ALL vision processing. Method B
 */
void Vision::visAdaptiveProcessing()
{

    /* abort if there is no image. */
    if ( visCvRaw->imageData==NULL )
    {
        printf("visCvRaw is NULL!\n");
        return;
    }

    /* Do vision processing */
    {

        /* generate visCvAdapt img */
        Adapt();
        //cvDilate(visCvAdapt, visCvAdapt, NULL, 1);

        /* shrink visCvAdapt img to 320x240 */
        //cvResize(visCvAdapt, visCvThresh, CV_INTER_LINEAR);
        //cvErode(visCvThresh, visCvThresh, NULL, 1);

        /* generate visCvPath */
        //visGenPath(visCvThresh);

        /* scan high in img */
        //int x,y;
        //robotWidthScan(visCvPath,x,y);

        /* setup navigation line that sweep across screen and updates goal. */
        //visSweeperLines(goal_near);

        /* update return goal */
       // CvtPixToGoal(goal);

    } //end vis processing

}

