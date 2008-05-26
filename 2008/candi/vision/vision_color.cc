#include "vision_color.h"

#include "vision_line_blobber.h"
#include "vision.h"
#include "vision_util.h"	// for HSB, HSL
#include <math.h>		// for sqrt
#include <stdlib.h>		// for abs

// ------------------------------------------------------------------------

// If 0, HSB.brightness is used to detect white
// If 1, HSL.lightness is used instead
#define USE_LIGHTNESS_INSTEAD_OF_BRIGHTNESS 1

/* higher values look for brighter orange */
// XXX: chosen default value is arbitrary; subsequent adjustment recommended
const int ORANGE_PIXEL_DETECTION_THRESHOLD =
	/*new IntFilterParam("Orange Pixels (Red-Green) - Threshold (High)", -255, 255,*/ 30;

/* lower values look for cleaner (less dirty) white */
// >=90 for low-light conditions     bigger->more red
const int WHITE_PIXEL_SATURATION_THRESHOLD =
	/*new IntFilterParam("White Pixel Saturation - Threshold (Low)", 0, 255,*/ 75;//85; //50;

/* HSB only: higher values look for brighter white */
// >=40 to eliminate black
const int WHITE_PIXEL_BRIGHTNESS_THRESHOLD =
	/*new IntFilterParam("White Pixel Brightness - Threshold (High)", 0, 255,*/ 100; //60;

/* HSL only: higher values look for brighter white */
// >=40 to eliminate black   bigger->less blue
const int WHITE_PIXEL_LIGHTNESS_THRESHOLD =
	/*new IntFilterParam("White Pixel Lightness - Threshold (High)", 0, 255,*/ 120; //140; //100; //60;


const Pixel ORANGE_PIXEL_ANNOTATION_COLOR = Pixel(255, 128, 0);		// bright orange
const Pixel WHITE_PIXEL_ANNOTATION_COLOR = Pixel(255, 255, 255);	// bright white
//const Pixel YELLOW_PIXEL_ANNOTATION_COLOR = Pixel(0, 255, 255);	// bright yellow???
const Pixel AMBIGUOUS_PIXEL_ANNOTATION_COLOR = Pixel(130, 62, 240);	// bright purple

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

const int YELLOW_PIXEL_TARGET_HSL_HUE = 42;
const int YELLOW_PIXEL_TARGET_HSL_SATURATION = 255;
const int YELLOW_PIXEL_TARGET_HSL_LIGHTNESS = 127;

// increase this to force the hue to more closely correspond to the target yellow hue
const double YELLOW_PIXEL_HUE_SIGNIFICANCE = 2.5;

const int YELLOW_PIXEL_TARGET_RADIUS = 64;

// ------------------------------------------------------------------------

// public
Buffer2D<bool> pixelIsOrange;
Buffer2D<bool> pixelIsWhite;
//Buffer2D<bool> pixelIsYellow;



// public
Buffer2D<Pixel> visWhiteDetectionCalibration;
Buffer2D<Pixel> visWhiteCondition;

// public
Buffer2D<unsigned char> visRedMinusGreen;

// public
Buffer2D<u8> visHSBHue;
Buffer2D<u8> visHSBSaturation;
Buffer2D<u8> visHSBBrightness;

// public
Buffer2D<u8> visHSLHue;
Buffer2D<u8> visHSLSaturation;
Buffer2D<u8> visHSLLightness;

// private
Buffer2D<int> pixelOrangeness;
Buffer2D<HSB> pixelHSB;
Buffer2D<HSL> pixelHSL;

// ------------------------------------------------------------------------

bool pixelIsOrange_calc(int off);
bool pixelIsWhite_calc(int off);
inline bool pixelIsWhite_calc_condition1(int off);
inline bool pixelIsWhite_calc_condition2(int off);
//bool pixelIsYellow_calc(int off);

Buffer2D<bool> pixelIsWhiteTemp;
bool pixelIsYellow_calcFromHSL(HSL hsl);

// ------------------------------------------------------------------------
void visGenPath();
Buffer2D<Pixel> visPathView;
int checkPaulBlobPixel(int x, int y);
void scanFillLeft(int middleX, int y, int goodFirst, int end);
void scanFillRight(int middleX, int y, int goodFirst, int end);


// ------------------------------------------------------------------------

/*		//old old old
void visGenPath(void){
	int width = visRaw.width;
	int height = visRaw.height;
	int good = 1;
	int black=0;
	visPathView.resize(width, height);

	Pixel p;
	Pixel p2;
	
	// scan up in columns; white = path; black = bad
	for(int x = 0; x < width; x++){
	
		good = 1;
		black = 6;
		
		for(int y = height-1; y >=0 ; y--){
		
			p = paulBlob.get(x,y);
		
			if(good){
			
				// 200: from vision line blobber
				if (p.red==200 && p.green==0 && p.blue==0){
					good = 0;
					p2.red=p2.green=p2.blue=0;
					visPathView.set(x,y,p2);	
				}
				// orange from shader
				else if (p.red==255 && p.green==128 && p.blue==0){
					good = 0;
					p2.red=p2.green=p2.blue=0;
					visPathView.set(x,y,p2);	
				}
				// black from transform
				else if (p.red==0 && p.green==0 && p.blue==0){
					black--;
					p2.red=p2.green=p2.blue=0;
					visPathView.set(x,y,p2);						
				}				
				// probably good
				else{
					p2.red=p2.green=p2.blue=255;
					visPathView.set(x,y,p2);
					good=1;
				}
				
				
			}
			// skip to top
			else{
				if(black<=0){ // not all transform black is totally black
					black=6;
					good=1;
				}else{
					good=0;
					p2.red=p2.green=p2.blue=0;
					visPathView.set(x,y,p2);				
				}

			}
			
		}//y
	
	}//x
	
	
}//visGenPath
*/


void visGenPath(void){
	int width = visRaw.width;
	int height = visRaw.height;
	int goodFirst = 1;
	int x = width/2;
	visPathView.resize(width, height);

	
	// scan bottom to top in rows; white = path; black = bad	
	for(int y = height-1; y >=0 ; y--){
		if(checkPaulBlobPixel(x,y)){	//check starting point in middle
			goodFirst=1;
		}
		else{
			goodFirst=0;
		}
		
		//scan left then right & generate visPathView image
		scanFillLeft(x,y,goodFirst,0);
		scanFillRight(x,y,goodFirst,width);		
			
	}//y

	
	
}//visGenPath


int checkPaulBlobPixel(int x, int y){
	int good;
	Pixel p = paulBlob.get(x,y);

	// 200: from vision line blobber
	if (p.red==200 && p.green==0 && p.blue==0){
		good = 0;	
	}
	// orange from shader
	else if (p.red==255 && p.green==128 && p.blue==0){
		good = 0;	
	}
	// black from transform
	else if (p.red==0 && p.green==0 && p.blue==0){
		good = 0;					
	}
	// probably good
	else{
		good = 1;
	}
	
	return good;
}

/*this function scans from the center of the paulBlob image to end
	checking for good / bad pixels, setting the visPathView image accordingly */
void scanFillLeft(int middleX, int y, int goodFirst, int end){
	int x=middleX;	
	int good;
	Pixel p2;
	
	if (goodFirst){		//starting pixel is good
		good=1;
		for(;x>=end;x--){	//scan left and check
			if(good){
				if(checkPaulBlobPixel(x,y)){
					//set good
					p2.red=p2.green=p2.blue=255;
					visPathView.set(x,y,p2);
				}
				else{
					//set bad
					good=0;
					p2.red=p2.green=p2.blue=0;
					visPathView.set(x,y,p2);
				}
			}
			else{		//all the rest are bad
				//set bad 
				p2.red=p2.green=p2.blue=0;
				visPathView.set(x,y,p2);
			}
		}
	}
	else {		//starting pixel is bad
		good=2;
		for(;x>=0;x--){	//scan left and check
			if(good==2){	//in bad spot, check for good spot
				if(checkPaulBlobPixel(x,y)){
					//set good
					good=1;
					p2.red=p2.green=p2.blue=255;
					visPathView.set(x,y,p2);
				}
				else{
					//set bad
					p2.red=p2.green=p2.blue=0;
					visPathView.set(x,y,p2);
				}
			}
			else if (good==1){	//a good spot appeared, check for bad again
				if(checkPaulBlobPixel(x,y)){
					//set good
					p2.red=p2.green=p2.blue=255;
					visPathView.set(x,y,p2);
					}
				else{
					//set bad
					good=0;
					p2.red=p2.green=p2.blue=0;
					visPathView.set(x,y,p2);
				} 
			}
			else{ //all the rest are bad
				//set bad
				p2.red=p2.green=p2.blue=0;
				visPathView.set(x,y,p2);
			}
		}		
	}
}
/*this function scans from the center of the paulBlob image to end
	checking for good / bad pixels, setting the visPathView image accordingly */
void scanFillRight(int middleX, int y, int goodFirst, int end){
	int x=middleX;	
	int good;
	Pixel p2;
	
	if (goodFirst){		//starting pixel is good
		good=1;
		for(;x<end;x++){	//scan right and check
			if(good){
				if(checkPaulBlobPixel(x,y)){
					//set good
					p2.red=p2.green=p2.blue=255;
					visPathView.set(x,y,p2);
				}
				else{
					//set bad
					good=0;
					p2.red=p2.green=p2.blue=0;
					visPathView.set(x,y,p2);
				}
			}
			else{		//all the rest are bad
				//set bad 
				p2.red=p2.green=p2.blue=0;
				visPathView.set(x,y,p2);
			}
		}
	}
	else {		//starting pixel is bad
		good=2;
		for(;x>=0;x--){	//scan right and check
			if(good==2){	//in bad spot, check for good spot
				if(checkPaulBlobPixel(x,y)){
					//set good
					good=1;
					p2.red=p2.green=p2.blue=255;
					visPathView.set(x,y,p2);
				}
				else{
					//set bad
					p2.red=p2.green=p2.blue=0;
					visPathView.set(x,y,p2);
				}
			}
			else if (good==1){	//a good spot appeared, check for bad again
				if(checkPaulBlobPixel(x,y)){
					//set good
					p2.red=p2.green=p2.blue=255;
					visPathView.set(x,y,p2);
					}
				else{
					//set bad
					good=0;
					p2.red=p2.green=p2.blue=0;
					visPathView.set(x,y,p2);
				} 
			}
			else{ //all the rest are bad
				//set bad
				p2.red=p2.green=p2.blue=0;
				visPathView.set(x,y,p2);
			}
		}		
	}
}


// public
void visClassifyPixelsByColor(void) {
	/* 
	 * Calculate the "orangeness", saturation, and brightness of each pixel:
	 * > pixelOrangeness(PIXEL) = PIXEL.red - PIXEL.green
	 * > pixelHSB = RGBtoHSB(PIXEL)
	 * 
	 * These values are used when determining the pixel's categorical color
	 * (orange, yelow, white, or a mix).
	 */
	pixelOrangeness.resize(visRaw.width, visRaw.height);
	pixelHSB       .resize(visRaw.width, visRaw.height);
	pixelHSL       .resize(visRaw.width, visRaw.height);
	for (int i=0, n=visRaw.numElements(); i<n; i++) {
		pixelOrangeness.data[i] = visRaw[i].red - visRaw[i].green;
		pixelHSB[i] = RGBtoHSB(visRaw[i]);
		pixelHSL[i] = RGBtoHSL(visRaw[i]);
	}
	
	/*
	 * Categorize the pixels by color.
	`*/
	pixelIsOrange.resize(visRaw.width, visRaw.height);
	pixelIsWhite.resize(visRaw.width, visRaw.height);
	//pixelIsYellow.resize(visRaw.width, visRaw.height);
	for (int i=0, n=visRaw.numElements(); i<n; i++) {
		pixelIsOrange[i] = pixelIsOrange_calc(i);
		pixelIsWhite[i] = pixelIsWhite_calc(i);
		//pixelIsYellow[i] = pixelIsYellow_calc(i);
	}
	
	// Filter out white pixels that are not connected to
	// at least one other white pixel that is surrounded by other white pixels
	visBlobLines();
	for (int i=0, n=pixelIsWhite.numElements(); i<n; i++) {
		pixelIsWhite[i] &= !whiteFilterMask[i];
	}
	
	#if 0
		/*
		 * Apply a noise filter to white detection
		 */
		// XXX: unhack me please!
		#define img pixelIsWhiteTemp
		int w = pixelIsWhite.width;
		int h = pixelIsWhite.height;
		pixelIsWhiteTemp.copyFrom(pixelIsWhite);
		for(int x=1;x<w-1;x++){
			for(int y=1;y<h-1;y++) {
				/*eight neighbor check					*/
				/*done in order of best short circuit	*/
				if(img.data[x+y*w]&&				
					img.data[x+1+y*w]&&				
					img.data[x-1+y*w]&&		
					img.data[x+(y+1)*w]&&	
					img.data[x+1+(y+1)*w]&&	
					img.data[x-1+(y+1)*w]&&	
					img.data[x+(y-1)*w]&&	
					img.data[x+1+(y-1)*w]&&	
					img.data[x-1+(y-1)*w])
				{
					pixelIsWhite.data[x+y*w] = 1;
				} else {
					pixelIsWhite.data[x+y*w] = 0;
				}
			}
		}
	#endif
	
	//create CHRIS PLAN VIEW
	visGenPath();
	
}

void visAnnotatePixelColors(Buffer2D<Pixel>& imageToAnnotate) {
	bool isOrange,isWhite,isAmbiguous;
	int numColorsPixelClassifiedAs;

	for (int i=0, n=imageToAnnotate.numElements(); i<n; i++) {
		isOrange = pixelIsOrange[i];
		isWhite = pixelIsWhite[i];
		//bool isYellow = pixelIsYellow[i];
		
		numColorsPixelClassifiedAs = 0;
		if (isOrange) numColorsPixelClassifiedAs++;
		if (isWhite) numColorsPixelClassifiedAs++;
		//if (isYellow) numColorsPixelClassifiedAs++;
		
		isAmbiguous = (numColorsPixelClassifiedAs >= 2);
		if (isAmbiguous) {
			imageToAnnotate[i] = AMBIGUOUS_PIXEL_ANNOTATION_COLOR;
		} else {
			if (isOrange)
				imageToAnnotate[i] = ORANGE_PIXEL_ANNOTATION_COLOR;
			if (isWhite)
				imageToAnnotate[i] = WHITE_PIXEL_ANNOTATION_COLOR;
			//if (isYellow)
			//	imageToAnnotate[i] = YELLOW_PIXEL_ANNOTATION_COLOR;
		}
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void visCreateHSBViews(void) {
	visHSBHue.resize(visRaw.width, visRaw.height);
	visHSBSaturation.resize(visRaw.width, visRaw.height);
	visHSBBrightness.resize(visRaw.width, visRaw.height);
	
	for (int i=0, n=visRaw.numElements(); i<n; i++) {
		HSB curHSB = pixelHSB[i];
		visHSBHue[i] = curHSB.hue;
		visHSBSaturation[i] = curHSB.saturation;
		visHSBBrightness[i] = curHSB.brightness;
	}
}

void visCreateHSLViews(void) {
	visHSLHue.resize(visRaw.width, visRaw.height);
	visHSLSaturation.resize(visRaw.width, visRaw.height);
	visHSLLightness.resize(visRaw.width, visRaw.height);
	
	for (int i=0, n=visRaw.numElements(); i<n; i++) {
		HSL curHSL = pixelHSL[i];
		visHSLHue[i] = curHSL.hue;
		visHSLSaturation[i] = curHSL.saturation;
		visHSLLightness[i] = curHSL.lightness;
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void visCreateWhiteCalibrationViewUsingColor(Pixel protoColor) {
	visCreateWhiteCalibrationViewUsingHue( RGBtoHSB(protoColor).hue );
}

void visCreateWhiteCalibrationViewUsingHue(u8 protoHue) {
	// Resize 'visWhiteDetectionCalibration' to match the size of the input image
	visWhiteDetectionCalibration.resizeToMatch(visRaw);
	int width = visWhiteDetectionCalibration.width;
	int height = visWhiteDetectionCalibration.height;
	
#if 1
	u8 minSaturation = (u8) 0;
	u8 maxSaturation = (u8) WHITE_PIXEL_SATURATION_THRESHOLD;
	#if USE_LIGHTNESS_INSTEAD_OF_BRIGHTNESS
		u8 minBrightness = (u8) WHITE_PIXEL_LIGHTNESS_THRESHOLD;
	#else
		u8 minBrightness = (u8) WHITE_PIXEL_BRIGHTNESS_THRESHOLD;
	#endif
	u8 maxBrightness = (u8) 255;
#else
	u8 minSaturation = (u8) 0;
	u8 maxSaturation = (u8) 255;
	u8 minBrightness = (u8) 0;
	u8 maxBrightness = (u8) 255;
#endif
	for (int y=0; y<height; y++) {
		u8 curBrightness = Pixel::fadeBetween(
			minBrightness, maxBrightness,
			y, height-1);
		
		for (int x=0; x<width; x++) {
			u8 curSaturation = Pixel::fadeBetween(
				minSaturation, maxSaturation,
				x, width-1);
			
			visWhiteDetectionCalibration.set(
				x, y,
#if USE_LIGHTNESS_INSTEAD_OF_BRIGHTNESS
				HSLtoRGB(HSL(protoHue, curSaturation, curBrightness))
#else
				HSBtoRGB(HSB(protoHue, curSaturation, curBrightness))
#endif
				);
		}
	}
}

// DEBUG
// XXX: This is now a yellow calibration view masquerading as HSL view masquerading as a white calibration view
void visCreateHSLColorSpaceView(u8 protoHue) {
	// Resize 'visWhiteDetectionCalibration' to match the size of the input image
	visWhiteDetectionCalibration.resizeToMatch(visRaw);
	int width = visWhiteDetectionCalibration.width;
	int height = visWhiteDetectionCalibration.height;
	
	u8 minSaturation = (u8) 0;
	u8 maxSaturation = (u8) 255; //WHITE_PIXEL_SATURATION_THRESHOLD;
	u8 minLightness = (u8) 0; //WHITE_PIXEL_BRIGHTNESS_THRESHOLD;
	u8 maxLightness = (u8) 255;
	
	bool looksLikeYellow;
	
	for (int y=0; y<height; y++) {
		u8 curLightness = Pixel::fadeBetween(
			minLightness, maxLightness,
			y, height-1);
		
		for (int x=0; x<width; x++) {
			u8 curSaturation = Pixel::fadeBetween(
				minSaturation, maxSaturation,
				x, width-1);
			
			looksLikeYellow = pixelIsYellow_calcFromHSL(
				HSL(protoHue, curSaturation, curLightness));
			
			Pixel curColor = HSLtoRGB(HSL(protoHue, curSaturation, curLightness));
			if (!looksLikeYellow) {
				// Dim non-yellow pixels
				curColor.red /= 2;
				curColor.green /= 2;
				curColor.blue /= 2;
			}
			visWhiteDetectionCalibration.set(
				x, y,
				curColor );
		}
	}
}

// DEBUG
void visTestRGBtoHSLConversions(void) {
	visTestViewContent.resizeToMatch(visRaw);
	
	for (int i=0, n=visRaw.numElements(); i<n; i++) {
		visTestViewContent[i] = HSLtoRGB(RGBtoHSL(visRaw[i]));
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void visCreateWhiteConditionView(void)
{
	visWhiteCondition.resizeToMatch(visRaw);
	
	bool condition1,condition2;
	
	for (int i=0, n=visRaw.numElements(); i<n; i++) {
		condition1 = pixelIsWhite_calc_condition1(i);
		condition2 = pixelIsWhite_calc_condition2(i);
		
		Pixel& curColor = visWhiteCondition[i];
		if (condition1) {
			if (condition2) {
				curColor = Pixel(255,255,255);	// white
			} else {
				// passes saturation, but brightness/lightness too low
				curColor = Pixel(255,0,0);	// red
			}
		} else {
			if (condition2) {
				// passed brightness/lightness, but saturation too high
				curColor = Pixel(0,0,255);	// blue
			} else {
				curColor = Pixel(0,0,0);	// black
			}
		}
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void visCreateRedMinusGreenView(void) {
	visRedMinusGreen.resizeToMatch(visRaw);
	int value;
	for (int i=0, n=visRaw.numElements(); i<n; i++) {
		Pixel p = visRaw[i];
		value = p.red - p.green;	// -255 to 255
		visRedMinusGreen[i] = value/2 + 128;
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// private
bool pixelIsOrange_calc(int off) {
	return (pixelOrangeness[off] >= ORANGE_PIXEL_DETECTION_THRESHOLD);
}

// private
bool pixelIsWhite_calc(int off) {
	return 
		pixelIsWhite_calc_condition1(off) &&
		pixelIsWhite_calc_condition2(off);
}

// private
inline bool pixelIsWhite_calc_condition1(int off) {
	return
		(pixelHSB[off].saturation <= WHITE_PIXEL_SATURATION_THRESHOLD);
}

// private
inline bool pixelIsWhite_calc_condition2(int off) {
	return
#if USE_LIGHTNESS_INSTEAD_OF_BRIGHTNESS
		(pixelHSL[off].lightness >= WHITE_PIXEL_LIGHTNESS_THRESHOLD);
#else
		(pixelHSB[off].brightness >= WHITE_PIXEL_BRIGHTNESS_THRESHOLD);
#endif
}

/*
// private
bool pixelIsYellow_calc(int off) {
	// TODO: implement
	throw QString("pixelIsYellow: not yet implemented");
}
*/

// private
bool pixelIsYellow_calcFromHSL(HSL hsl) {
	int deltaHue = (int) ((double) abs(((int) hsl.hue) - YELLOW_PIXEL_TARGET_HSL_HUE)
	                      * YELLOW_PIXEL_HUE_SIGNIFICANCE );	// bias
	int deltaSaturation = abs(((int) hsl.saturation) - YELLOW_PIXEL_TARGET_HSL_SATURATION);
	int deltaLightness = abs(((int) hsl.lightness) - YELLOW_PIXEL_TARGET_HSL_LIGHTNESS);
	
	double distFromYellow = sqrt( (deltaHue * deltaHue) + 
	                              (deltaSaturation * deltaSaturation) +
	                              (deltaLightness * deltaLightness) );
	bool looksLikeYellow = (distFromYellow < YELLOW_PIXEL_TARGET_RADIUS);
	return looksLikeYellow;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

