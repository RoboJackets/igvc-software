#include "vision.h"
#include <stdio.h>
#include "vision_processing.h"
#include "XmlConfiguration.h"
#include "image_buffers.h"



// -----------------------------------------------------------------------------


int satThreshold;
int hueThreshold;
//int goalx,goaly;
//Point2D<int> goal;
// goal set by robotWidthScan()
Point2D<int> goal_far;
// goal set by visSweeperLines()
Point2D<int> goal_near;

// -----------------------------------------------------------------------------



/* Performs ALL vision processing. */
void visProcessFrame(Point2D<int>& goal) {

    /* Abort if there is no image. */
    if ( visCvRaw->imageData==NULL ) {
        printf("visCvRaw is NULL!\n");
        return;
    }

    /* Do vision processing (with opencv images & vision_processing.cpp) */
    {
        /* no memory leaks (cloning image) */
        if (visCvDebug) cvReleaseImage(&visCvDebug);

        /* copy raw image for debug drawing */
        visCvDebug = cvCloneImage(visCvRaw);

        /* split images into channels */
        //GetRGBChannels();
        GetHSVChannels();

        /* threshold saturation
         * (320x240) */
        cvSmooth(visCvSaturation, visCvSaturation);
        ThresholdImage(visCvSaturation, visCvSaturation, satThreshold);
        cvDilate(visCvSaturation, visCvSaturation, NULL, 1);

        /* threshold hue
         * (320x240) */
        cvSmooth(visCvHue, visCvHue);
        ThresholdImage(visCvHue, visCvHue, hueThreshold);
        cvDilate(visCvHue, visCvHue, NULL, 1);

        /* or the images together
         * (320x240) */
        cvOr(visCvSaturation, visCvHue, visCvThresh);
        cvDilate(visCvThresh, visCvThresh, NULL, 1);

        /* make white=good & black=bad
         * (320x240) */
        cvNot(visCvThresh, visCvThresh);

        /* generate path image, with white=path & black=bad
         * (320x240) */
        visGenPath(visCvThresh);


        /* find next goal for robot by scanning up visCvPath Image, and set the goal.
         * (320x240) */
        robotWidthScan(visCvPath,goal_far.x,goal_far.y);	// returns -1 on error

        /* sent goal to planner to get the 'planned' next goal */
        if (goal_far.y != -1) {
            //visPlanPath(visCvPath, goalx, goaly);	// TODO: THIS IS SO SLOW
        }

        /* setup navigation line that sweep across screen and update goal. */
        visSweeperLines(goal_near);

        /* update return goal
         * NOTE: goal is in the frame of 320x240 !! */
        if (goal_far.y >= visCvPath->height/2) {
            goal = goal_near;	// can't see very far
        } else {
            goal = goal_far;	// can see far off
            goal.x = visCvPath->width - goal.x;
        }

        /* convert goal to heading:
         * x = rotational speed ; range = (-128,127)
         * y = forward speed    ; range = (0,255)
         */
        {
            /* remember, goal is in 320x240 coordinates */

            // rotation (0 = go straight)
            goal.x = (visCvPath->width/2 - goal.x) * (256) / (visCvPath->width) ; 
            
            // fwd speed
            goal.y = (visCvPath->height  - goal.y) * (256) / (visCvPath->height);

            // Debug print
            printf("heading: rot: %d 	fwd: %d \n",goal.x,goal.y);
        }




    } //end main




}//end vision processing


void ConvertAllImageViews() {

    switch (trackbarVal) {
    case 0:
        cvShowImage("display", visCvRaw);
        break;
    case 1:
        cvShowImage("display", visCvDebug);
        break;
    case 2:
        cvShowImage("display", visCvPath);
        break;
    case 3:
        cvShowImage("display", visCvThresh);
        break;
    case 4:
        cvShowImage("display", visCvHue);
        break;
    case 5:
        cvShowImage("display", visCvSaturation);
        break;
    case 6:
        cvShowImage("display", visCvHSV);
        break;
//        	case 7:
//        		cvShowImage("display", );
//        		break;
//        	case 8:
//        		cvShowImage("display", );
//        		break;
    }
    cvWaitKey(10);

}


void LoadVisionXML() {

    // load xml settings
    XmlConfiguration cfg("Config.xml");
    satThreshold = cfg.getInt("satThresh");
    hueThreshold = cfg.getInt("hueThresh");

    if (satThreshold==-1 || hueThreshold==-1)
        printf("Vision settings NOT loaded! \n");
    else
        printf("Vision settings loaded \n");

    printf("sat %d  hue %d \n",satThreshold,hueThreshold);
}


/* for selecting images to display in the opencv window */
int trackbarVal;
char* names[] = {"raw","debug","path","thresh","hue","sat","hsv","?","?"};
void trackbarHandler(int pos) {
    printf("pos = %d \n", pos);
    printf("view = %s \n", names[pos]);
}
