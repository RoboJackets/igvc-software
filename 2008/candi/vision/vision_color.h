#ifndef _VISION_COLOR_H_
#define _VISION_COLOR_H_

#include "Buffer2D.h"
#include "Pixel.h"

// OUTPUT
extern Buffer2D<bool> pixelIsOrange;
extern Buffer2D<bool> pixelIsWhite;
//extern Buffer2D<bool> pixelIsYellow;

// OUTPUT: View used to calibrate white-color detection.
extern Buffer2D<Pixel> visWhiteDetectionCalibration;
extern Buffer2D<Pixel> visWhiteCondition;

extern Buffer2D<unsigned char> visRedMinusGreen;

extern Buffer2D<u8> visHSBHue;
extern Buffer2D<u8> visHSBSaturation;
extern Buffer2D<u8> visHSBBrightness;

extern Buffer2D<u8> visHSLHue;
extern Buffer2D<u8> visHSLSaturation;
extern Buffer2D<u8> visHSLLightness;

//---------

//---------

/**
 * Determines which pixels in 'visRaw' are orange, white, yellow, or some
 * combination of multiple colors.
 * 
 * Stores results in the global buffers 'pixelIsOrange',
 * 'pixelIsWhite', and 'pixelIsYellow'.
 */
void visClassifyPixelsByColor(void);

/**
 * Annotates the pixels of 'imageToAnnotate' whose colors have been
 * identified with bright colors. Those pixels of 'imageToAnnotate'
 * whose colors have not been identified are left alone.
 * 
 * It is assumed that the dimensions of 'imageToAnnotate'
 * match the dimensions of 'visRaw'.
 */
void visAnnotatePixelColors(Buffer2D<Pixel>& imageToAnnotate);

/**
 * Takes an initial color and then generates a picture that shows
 * all forms of that color which would be considered white
 * according to the current calibration parameters.
 * 
 * The x-axis of the output image is saturation and
 * the y-axis of the output image is brightness.
 */
void visCreateWhiteCalibrationViewUsingColor(Pixel protoColor);
void visCreateWhiteCalibrationViewUsingHue(u8 protoHue);

void visCreateWhiteConditionView(void);

void visCreateRedMinusGreenView(void);

// DEBUG
void visCreateHSLColorSpaceView(u8 protoHue);

// DEBUG
void visTestRGBtoHSLConversions(void);

void visCreateHSBViews(void);
void visCreateHSLViews(void);

//void processRamps(void);

void blankColredBarrels();
void updatePixelColors(void);

#endif
