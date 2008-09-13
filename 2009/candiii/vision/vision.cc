#include "vision.h"
#include <stdio.h>
#include "vision_processing.h"
#include "XmlConfiguration.h"
#include "image_buffers.h"



// -----------------------------------------------------------------------------


int satThreshold;
int hueThreshold;
//int goalx,goaly;
Point2D<int> goal;

// -----------------------------------------------------------------------------



/* Performs ALL vision processing. */
void visProcessFrame() {

    /* Abort if there is no image. */
    if ( visCvRaw->imageData==NULL ) {
        printf("visCvRaw is NULL!\n");
        return;
    }

    /* Do vision processing (with opencv images & vision_processing.cpp) */
    {

        /* copy raw image for debug drawing */
        visCvDebug = cvCloneImage(visCvRaw);

        /* split images into channels */
        //GetRGBChannels();
        GetHSVChannels();

        /* threshold saturation */
        cvSmooth(visCvSaturation, visCvSaturation);
        ThresholdImage(visCvSaturation, visCvSaturation, satThreshold);
        cvDilate(visCvSaturation, visCvSaturation, NULL, 1);

        /* threshold hue */
        cvSmooth(visCvHue, visCvHue);
        ThresholdImage(visCvHue, visCvHue, hueThreshold);
        cvDilate(visCvHue, visCvHue, NULL, 1);

        /* or the images together */
        cvOr(visCvSaturation, visCvHue, visCvThresh);
        cvDilate(visCvThresh, visCvThresh, NULL, 1);

        /* make white=good & black=bad */
        cvNot(visCvThresh, visCvThresh);

        /* generate path image, with white=path & black=bad */
        visGenPath(visCvThresh);


        /* find next goal for robot by scanning up visCvPath Image,
         * and set the goal */
        robotWidthScan(visCvPath,goal.x,goal.y);	// returns -1 on error

        /* sent goal to planner to get the 'planned' next goal */
        if(goal.y!=-1){
        	//visPlanPath(visCvPath, goalx, goaly);	// TODO: THIS IS SO SLOW
        }

		/* setup navigation line that sweep across screen 
		 * and update goal */
		visSweeperLines(goal);


    }




}//end vision processing


void ConvertAllImageViews(){
	
        switch(trackbarVal){
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
void trackbarHandler(int pos){
	printf("pos = %d \n", pos);
}
