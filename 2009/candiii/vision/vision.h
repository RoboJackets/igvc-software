#ifndef _VISION_H_
#define _VISION_H_



/*
 * This file contains the robot's primary vision processing code main function.
 *   by: Chris McClanahan
 *
 */


// This is called each frame to do vision processing
void visProcessFrame();


// loads the thresholds for vision processing from the xml config file
void LoadVisionXML();

// trackbar value determines image view to display 
void ConvertAllImageViews();
// callback for trackbar
void trackbarHandler(int pos);
// trackbar value determines image view to display
extern int trackbarVal;






#endif // _VISION_H_

