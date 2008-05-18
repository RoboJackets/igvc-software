/*
 * This file is a direct port of the BarrelBlobFinder class (from Java)
 * included in the standard distribution of ImageFilterDemo,
 * IGVC's vision algorithm prototyping program.
 */

#include "vision_barrels.h"

#include "vision.h"
#include "vision_color.h"
#include "Pixel.h"
#include "Buffer2D.h"
#include "Graphics.h"
#include <algorithm>	// for the STL sort(...) function

// ### FORWARD REFERENCES (C/C++ SPECIFIC) ###

int countNonignoredOrangePixels(int off, int len);
int countNonignoredWhitePixels(int off, int len);

void drawTaperToleranceForBarrel(Graphics& g, int x, int y, int width, int height);

// ### PARAMETERS ###

// This macro adjusts lengths expressed in pixels based on the expected input image size.
// The current definition assumes visRaw has width 360
//   but that the parameters are calibrated against the actual
//   size of the image transmitted by the DV camera, which is 720.
// (720/360 == 2)
#define PixelLength(pixLen) (pixLen / 2)

const Pixel BARREL_BOUND_COLOR = Pixel(255, 255, 0);		// Color.YELLOW
const int BARREL_BOUND_THICKNESS = 3;

//const Pixel BARREL_COUNTER_COLOR = Pixel(255, 255, 0);	// Color.YELLOW
const Pixel ORANGE_SCANLINE_COLOR = Pixel(255/2, 128/2, 0);	// darker orange???

/* distance between drawn heads-up-display (HUD) type symbols
 * and the corners of the output image */
const int HUD_PADDING_FROM_CORNER = 5;

//bool DEBUG_UPPER_EDGE_DETERMINATION = FALSE;

/* lower values tolerate less orange-pixel noise but give better results
 * when little orange-pixel noise is present */
// XXX: chosen default value is arbitrary; subsequent adjustment recommended
float barrelEdgeNoiseTolerance =
	/*new FloatFilterParam("Barrel Edge - Noise Tolerance", 0.00f, 0.50f,*/ 0.15f; //0.07f;

#define GROW_WIDTH_TO_COMPENSATE_FOR_BARREL_EDGE_NOISE_FILTER 1

/* higher values allow longer gaps between orange/white stripes,
 * tolerate larger highlights on the barrel,
 * tolerate larger deep shadows on the barrel,
 * but distinguish individual barrels in barricades less accurately
 * 
 * NOTE: Be careful not to set this parameter too high (especially when calibrating other parameters).
 */
// <=15 to avoid overriding white-stripe detection
// >=2 for low-light conditions (with shadows)
int maxStripeGapLength =
	/*new IntFilterParam("Maximum Stripe Gap Length", 0, Integer.MAX_VALUE,*/ PixelLength(5);

/* lower values look for thinner barrels, but are more confused by orange-pixel noise */
// <=60 to detect far away barrels
int numOrangePixelsInBarrelGirthThreshold =
	/*new IntFilterParam("Minimum Barrel Girth (Width + Noise)", 1, Integer.MAX_VALUE,*/ PixelLength(40);

/* lower values allow detection of very near and very far barrels,
 * but make it easier to incorrectly identify small *sections* of
 * barrels as being full-sized barrels */
// <=90 to detect far away barrels
int minBarrelHeight =
	/*new IntFilterParam("Minimum Barrel Height", 1, Integer.MAX_VALUE,*/ PixelLength(70);

/* higher values detect the upper-edge of barrels more accurately in the presense
 * of multiple barrels, but tolerate less tapering */
// XXX: chosen default value is arbitrary; subsequent adjustment recommended
float taperTolerance =
	/*new FloatFilterParam("Barrel Tapering % - Tolerated Threshold (High)", 0.00f, 1.00f,*/ 0.70f;

/* higher values detect the width of barrels more accurately */
// XXX: chosen default value is arbitrary; subsequent adjustment recommended
int maxHeightOfBarrelCurvature =
	/*new IntFilterParam("Maximum Height of Barrel Curvature", 1, Integer.MAX_VALUE,*/ PixelLength(40);

typedef enum {
	IETBSA_AVERAGE = 1,
	IETBSA_MINMAX = 2,
	IETBSA_PERCENTILE = 3
} IntervalEdgeToBarrelSidesAlgorithm;

/* defines the algorithm used to find barrel edges,
 * given a series of horizontal orange intervals */
IntervalEdgeToBarrelSidesAlgorithm chosenIntervalEdgeToBarrelSidesAlgorithm = IETBSA_PERCENTILE;

/* if "chosenIntervalEdgeToBarrelSidesAlgorithm" is "IETBSA_PERCENTILE",
 * defines the percentile that is used to determine the left/right edge of barrels
 * from a list of orange-pixel intervals;
 * lower values tolerate fewer crazy intervals but give better results on average;
 * NOTE: If you set this to 0.00f, the percentile mode acts the same as the min-max mode.
 */
float intervalEdgeVariationTolerance = 0.50f; //0.10f;

// ### ALGORITHM ###

/*
Buffer2D<int> pixelOrangeness;
Buffer2D<int> pixelSaturation;
Buffer2D<int> pixelBrightness;

Buffer2D<bool> pixelIsOrange;
Buffer2D<bool> pixelIsWhite;
*/

// OUTPUT
// public
Buffer2D<Pixel> visBarrels;
QList< Rectangle<int> > barrelBoundList;
Buffer2D<bool> pixelIsWithinBarrel;

// public
void visFindBarrels(void) {
	Buffer2D<Pixel>& inPixels = visRaw;
	Buffer2D<Pixel>& outPixels = visBarrels;
	int width = visRaw.width;
	int height = visRaw.height;
	int numPixels = width*height;

	// Clear previous output from output buffers
	barrelBoundList.clear();
	
	// Initialize
	pixelIsWithinBarrel.resize(inPixels.width, inPixels.height);
	for (int i=0, n=pixelIsWithinBarrel.numElements(); i<n; i++) {
		pixelIsWithinBarrel[i] = false;
	}
	
	/* Prepare output image (#1) */
	Graphics g = Graphics(&outPixels);
	{
		outPixels.copyFrom(inPixels);

		/* Draw the pixels that were detected as orange/white */
		visAnnotatePixelColors(outPixels);
	}
	
	/* Begin scanning rows of the image for lower-edges of barrels, starting from the bottom */
	int numBarrelsDetected;
	{
		for (int y=height-1, rowOff=numPixels-width;
			 y>=0;
			 y--, rowOff-=width)
		{
			/* Count the number of "orange" pixels in the current row */
			int numOrangePixelsInRow = countNonignoredOrangePixels(rowOff, width);
			
			/* 
			 * If the number of orange pixels in the current row
			 * exceeds the barrel-detection threshold, assume that we have
			 * encountered the lower-edge of one or more barrels, and
			 * attempt to identify those barrels.
			 */
			if (numOrangePixelsInRow > numOrangePixelsInBarrelGirthThreshold) {
				int lowerBarrelEdgeY = y;
				
				/* 
				 * XXX: The following lower-barrel-edge isolation algorithm is simplified, 
				 *      in that it makes the assumption that the lower edges of more than
				 *      one barrel will never occur in the same row
				 * 
				 * Assume that we have hit the lower edge of *exactly one* barrel.
				 */
				
				/*
				 * Locate the left/right edges of the barrel, "shaving" off any orange-pixel "noise",
				 * using the following algorithm:
				 * 
				 * Locate the (barrelEdgeNoiseTolerance)% and the
				 * (1.0-barrelEdgeNoiseTolerance)% percentiles of the list of
				 * orange pixels in the current row and the "nearby" rows above it.
				 * (A row is considered "nearby" if it is within (maxHeightOfBarrelCurvature)
				 *  rows of the current row.)
				 * 
				 * (These percentile-intervals will be used in a subsequent step
				 *  to identify the left/right edges of the barrel.)
				 * 
				 * REMARK: The reason that "nearby" rows are assessed in addition to the
				 *         current row is that this yields a significantly more accurate
				 *         value for the width of the detected barrel in instances when
				 *         the bottom of the barrel appears curved (which is very common).
				 */
				int numNearbyRows = Math_min(maxHeightOfBarrelCurvature, y+1);
				int numHorizIntervals = 0;		// eventually this will hold the total number of intervals (<= numNearbyRows)
				int horizIntervalLowX[numNearbyRows];	// only the first "numHorizIntervals" elements of this array are valid
				int horizIntervalHighX[numNearbyRows];	// ditto
				for (int y2 = y, rowOff2 = rowOff, minY2 = y + 1 - numNearbyRows;
						y2 >= minY2;
						y2--, rowOff2-=width)
				{
					int numOrangePixelsInRow2 = countNonignoredOrangePixels(rowOff2, width);
					
					// BUGFIX: This case wasn't dealt with properly before
					if (numOrangePixelsInRow2 == 0) {
						continue;
					}
					
					/*
					 * Calculate the bounds of the interval of orange pixels
					 * in the current row.
					 * 
					 * This is done by locating the (barrelEdgeNoiseTolerance)% and the
					 * (1.0-barrelEdgeNoiseTolerance)% percentiles of the list of
					 * orange pixels in the current row.
					 */
					int lowPercentileOrangePixelX = 0;			// (default)
					int highPercentileOrangePixelX = width-1;	// (default)
					{
						int lowPercentileOrangePixelOrdinal = 
							(int) ((numOrangePixelsInRow2-1) * barrelEdgeNoiseTolerance);
						int highPercentileOrangePixelOrdinal =
							(int) ((numOrangePixelsInRow2-1) - lowPercentileOrangePixelOrdinal);
						// BUGFIX: "off" should be initialized to "rowOff2", NOT "rowOff"
						for (int x=0, off=rowOff2, orangePixCount=0; x<width; x++, off++) {
							if (pixelIsOrange[off] && !pixelIsWithinBarrel[off]) {
								if (orangePixCount == lowPercentileOrangePixelOrdinal) {
									lowPercentileOrangePixelX = x;
								}
								if (orangePixCount == highPercentileOrangePixelOrdinal) {
									highPercentileOrangePixelX = x;
								}
								
								// BUGFIX: Previously we were incrementing BEFORE
								//         checking whether the count matched an ordinal
								//         (which prevented ordinal #0 from ever being found)
								orangePixCount++;
							}
						}
					}
					
					// Draw the interval we just found
					g.setColor(ORANGE_SCANLINE_COLOR);
					g.drawHorizLine(y2, lowPercentileOrangePixelX, highPercentileOrangePixelX);
					
					// Record the high/low X values for the interval we just found
					horizIntervalLowX[numHorizIntervals] = lowPercentileOrangePixelX;
					horizIntervalHighX[numHorizIntervals] = highPercentileOrangePixelX;
					numHorizIntervals++;
				}
				
				/*
				 * Calculate the left/right edges of the barrel, based on
				 * the horizontal intervals of orange-pixels that were identified.
				 *
				 * The precise algorithm used to perform this calculation
				 * is controlled by a parameter.
				 */
				int leftBarrelEdgeX;
				int rightBarrelEdgeX;
				switch (chosenIntervalEdgeToBarrelSidesAlgorithm) {
				case IETBSA_AVERAGE: {
					/*
					 * leftBarrelEdgeX = <the average of the low-X values of the intervals>
					 * rightBarrelEdgeX = <the average of the high-X values of the intervals>
					 */
					int sumIntervalLowX = 0;
					int sumIntervalHighX = 0;
					for (int i=0; i<numHorizIntervals; i++) {
						int curIntervalLowX = horizIntervalLowX[i];
						int curIntervalHighX = horizIntervalHighX[i];
						
						sumIntervalLowX += curIntervalLowX;
						sumIntervalHighX += curIntervalHighX;
					}
					int avgIntervalLowX = sumIntervalLowX/numHorizIntervals;
					int avgIntervalHighX = sumIntervalHighX/numHorizIntervals;
					
					leftBarrelEdgeX = avgIntervalLowX;
					rightBarrelEdgeX = avgIntervalHighX;
					break;
				}
				case IETBSA_MINMAX: {
					/*
					 * leftBarrelEdgeX = <the minimum of the low-X values of the intervals>
					 * rightBarrelEdgeX = <the maximum of the high-X values of the intervals>
					 */
					int minIntervalLowX = width-1;
					int maxIntervalHighX = 0;
					for (int i=0; i<numHorizIntervals; i++) {
						int curIntervalLowX = horizIntervalLowX[i];
						int curIntervalHighX = horizIntervalHighX[i];
						
						if (curIntervalLowX < minIntervalLowX)
							minIntervalLowX = curIntervalLowX;
						if (curIntervalHighX > maxIntervalHighX)
							maxIntervalHighX = curIntervalHighX;
					}
					
					leftBarrelEdgeX = minIntervalLowX;
					rightBarrelEdgeX = maxIntervalHighX;
					break;
				}
				case IETBSA_PERCENTILE: {
					// Sort the low-X and high-X coordinates of the intervals
					std::sort(horizIntervalLowX, horizIntervalLowX + numHorizIntervals);
					std::sort(horizIntervalHighX, horizIntervalHighX + numHorizIntervals);
					
					// Find the values at the (intervalEdgeVariationTolerance)% percentile of the low-X values
					// and the (1.0-intervalEdgeVariationTolerance)% percentile of the high-X values
					int lowPercentileOrdinal = (int) ((numHorizIntervals-1) * intervalEdgeVariationTolerance);
					int highPercentileOrdinal = (numHorizIntervals-1) - lowPercentileOrdinal;
					
					int lowPercentileValue = horizIntervalLowX[lowPercentileOrdinal];
					int highPercentileValue = horizIntervalHighX[highPercentileOrdinal];
					
					leftBarrelEdgeX = lowPercentileValue;
					rightBarrelEdgeX = highPercentileValue;
					break;
				}
				default: {
					printf("vision_barrels.cc: invalid value for \"chosenIntervalEdgeToBarrelSidesAlgorithm\"\n");
					
					leftBarrelEdgeX = 0;
					rightBarrelEdgeX = width - 1;
					break;
				}
				}
				
				// Grow the detected barrel sides by approximately the amount
				// that was lost during the barrel-edge-noise-tolerance calculation
				if (GROW_WIDTH_TO_COMPENSATE_FOR_BARREL_EDGE_NOISE_FILTER) {
					//bigWidth*(1 - 2*barrelEdgeNoiseTolerance) == smallWidth
					
					int smallWidth = rightBarrelEdgeX - leftBarrelEdgeX + 1;
					int bigWidth = (int)(smallWidth/(1 - 2*barrelEdgeNoiseTolerance));
					int lengthToGrowSides = (bigWidth-smallWidth)/2;
					//printf("DEBUG: lengthToGrowSides=%d\n", lengthToGrowSides);
					
					#define min(a,b) (((a)<(b)) ? (a) : (b))
					#define max(a,b) (((a)>(b)) ? (a) : (b))
					leftBarrelEdgeX = max(leftBarrelEdgeX - lengthToGrowSides, 0);
					rightBarrelEdgeX = min(rightBarrelEdgeX + lengthToGrowSides, width-1);
				}
				
				int barrelWidth = rightBarrelEdgeX - leftBarrelEdgeX + 1;
				
				// Put dots at the bottom left and right corners of the potential barrel
				g.setColor(BARREL_BOUND_COLOR);
				g.drawPixel(leftBarrelEdgeX, lowerBarrelEdgeY);
				g.drawPixel(rightBarrelEdgeX, lowerBarrelEdgeY);
				
				/*
				 * Locate the upper edge of the barrel
				 * (while tolerating a certain amount of tapering and
				 *  while tolerating gaps up to a certain length).
				 * 
				 * Scan row-segments above the detected lower-barrel-edge,
				 * starting from the bottom and moving upward.
				 * 
				 * Stop scanning when either:
				 * 1) more than (maxStripeGapLength) row-segments are
				 *    encountered whose pixels are less than 
				 *    (taperTolerance)% orange/white 
				 * 2) the top of the image is encountered
				 */
				int upperBarrelEdgeY;
				{
					int lastSeenStripeUpperEdgeY = lowerBarrelEdgeY;		// (default)
					int numOrangeOrWhitePixelsInRowSegmentThreshold
						= (int) (barrelWidth * taperTolerance);
					for (int y2=lowerBarrelEdgeY, rowSegOff = rowOff+leftBarrelEdgeX, gapLength = 0;
						 (y2>=0) && (gapLength <= maxStripeGapLength);
						 y2--, rowSegOff-=width)
					{
						int numOrangePixelsInRowSegment = countNonignoredOrangePixels(rowSegOff, barrelWidth);
						int numWhitePixelsInRowSegment = countNonignoredWhitePixels(rowSegOff, barrelWidth);
						int numOrangeOrWhitePixelsInRowSegment = 
							numOrangePixelsInRowSegment +
							numWhitePixelsInRowSegment;
						
						if (numOrangeOrWhitePixelsInRowSegment < numOrangeOrWhitePixelsInRowSegmentThreshold) {
							/*
							 * Insufficent number of orange/white pixels found;
							 * we've either:
							 *  1) hit the upper-edge of the barrel
							 *  2) hit a gap between adjacent stripes
							 */
							gapLength++;
							
							/*
							if (DEBUG_UPPER_EDGE_DETERMINATION) {
								System.out.println(
									"DEBUG: "+numOrangeOrWhitePixelsInRowSegment+
										"("+numOrangePixelsInRowSegment+","+
										numWhitePixelsInRowSegment+") < "+
									numOrangeOrWhitePixelsInRowSegmentThreshold+" < "+
									barrelWidth);
							}
							*/
						} else {
							// Reset gap length to zero (since we no longer see a gap)
							gapLength = 0;
							
							// Record the position of the stripe that we currently see
							lastSeenStripeUpperEdgeY = y2;
						}
					}
					upperBarrelEdgeY = lastSeenStripeUpperEdgeY;
				}
				int barrelHeight = lowerBarrelEdgeY - upperBarrelEdgeY + 1;
				
				/*
				 * Ensure the identified "barrel" satisfies the minimum height requirement
				 */
				if (barrelHeight >= minBarrelHeight) {
					/*
					 * In subsequent scans, ignore the region of the image in which
					 * we identified the barrel.
					 */
					for (int y2=upperBarrelEdgeY, rowOff2 = (upperBarrelEdgeY*width) + leftBarrelEdgeX;
						 y2<=lowerBarrelEdgeY;
						 y2++, rowOff2+=width)
					{
						for (int x2=leftBarrelEdgeX, off2=rowOff2;
							 x2<=rightBarrelEdgeX;
							 x2++, off2++)
						{
							pixelIsWithinBarrel[off2] = true;
						}
					}
					
					// Record the bounds of the barrel that was found
					Rectangle<int> barrelBounds = /*(Rectangle<int>)*/ {
						leftBarrelEdgeX,	// x
						upperBarrelEdgeY,	// y
						barrelWidth,		// width
						barrelHeight		// height
					};
					barrelBoundList.append(barrelBounds);
				}
			}
		}
		
		numBarrelsDetected = barrelBoundList.size();
	}
	
	/* 
	 * Prepare output image (#2)
	 * 
	 * The output image consists of the input image,
	 * plus annotations that show where the boundary of each detected barrel is.
	 */
	{
		visAnnotateBarrelBounds(outPixels, true);
	}
	
	/*
	if (DEBUG_UPPER_EDGE_DETERMINATION) {
		System.out.println("DEBUG: done");
	}
	*/
}

// public
void visAnnotateBarrelBounds(Buffer2D<Pixel>& imageToAnnotate, bool showBarrelFinderParameters) {
	Graphics g = Graphics(&imageToAnnotate);
	//int width = imageToAnnotate.width;
	int height = imageToAnnotate.height;
	
	/*
	 * Draw the bounds of each detected barrel,
	 * along with some additional annotations
	 */
	g.setColor(BARREL_BOUND_COLOR);
	for (int i=0, n=barrelBoundList.size(); i<n; i++) {
		Rectangle<int> curBarrel = (Rectangle<int>) barrelBoundList[i];
		
		/* Draw the bounds of the barrel */
		int barrelX = curBarrel.x;
		int barrelY = curBarrel.y;
		int barrelWidth = curBarrel.width;
		int barrelHeight = curBarrel.height;
		for (int j=BARREL_BOUND_THICKNESS; j>0; j--) {
			/* 
			 * NOTE: The -1's on the width and height are to compensate for the way 
			 * that drawRect(...) determines the positions of the right/bottom
			 * edges of the rectangle to draw
			 */
			g.drawRect(barrelX, barrelY, barrelWidth-1, barrelHeight-1);

			barrelX++; barrelY++; barrelWidth-=2; barrelHeight-=2;
		}

		/* Draw a line representing "maxHeightOfBarrelCurvature" */
		if (showBarrelFinderParameters) {
			if (curBarrel.height > maxHeightOfBarrelCurvature) {
				g.drawHorizLine(
					curBarrel.y + curBarrel.height - maxHeightOfBarrelCurvature,
					curBarrel.x,
					curBarrel.x + curBarrel.width - 1);
			}
		}
		
		/* 
		 * Draw an interval of length "numOrangePixelsInBarrelGirthThreshold"
		 * on the bottom edge of the barrel.
		 */
		if (showBarrelFinderParameters) {
			g.drawVertLine(
				curBarrel.x + (curBarrel.width - numOrangePixelsInBarrelGirthThreshold)/2,
				curBarrel.y + curBarrel.height - 2*BARREL_BOUND_THICKNESS,
				curBarrel.y + curBarrel.height + 1*BARREL_BOUND_THICKNESS);
			g.drawVertLine(
				curBarrel.x + (curBarrel.width + numOrangePixelsInBarrelGirthThreshold)/2,
				curBarrel.y + curBarrel.height - 2*BARREL_BOUND_THICKNESS,
				curBarrel.y + curBarrel.height + 1*BARREL_BOUND_THICKNESS);
		}
		
		/*
		 * Draw an interval of length "minBarrelHeight" on the left edge of the barrel.
		 */
		if (showBarrelFinderParameters) {
			g.drawHorizLine(
				curBarrel.y + (curBarrel.height - minBarrelHeight)/2,
				curBarrel.x - 1*BARREL_BOUND_THICKNESS,
				curBarrel.x + 2*BARREL_BOUND_THICKNESS);
			g.drawHorizLine(
				curBarrel.y + (curBarrel.height + minBarrelHeight)/2,
				curBarrel.x - 1*BARREL_BOUND_THICKNESS,
				curBarrel.x + 2*BARREL_BOUND_THICKNESS);
		}
		
		/* Draw the effect of the "taperTolerance" parameter */
		if (showBarrelFinderParameters) {
			drawTaperToleranceForBarrel(g, curBarrel.x, curBarrel.y, curBarrel.width, curBarrel.height);
		}
	}
	
	/*
	 * Draw a prototypical minimal-size barrel in the 
	 * lower left-hand corner of the output image.
	 * 
	 * Also draw the effect of the "taperTolerance" parameter.
	 */
	if (showBarrelFinderParameters) {
		int miniBarrelX = HUD_PADDING_FROM_CORNER;
		int miniBarrelY = height - HUD_PADDING_FROM_CORNER - minBarrelHeight;
		g.drawRect_rational(
			miniBarrelX,
			miniBarrelY,
			numOrangePixelsInBarrelGirthThreshold,
			minBarrelHeight);
		
		drawTaperToleranceForBarrel(
			g,
			miniBarrelX,
			miniBarrelY,
			numOrangePixelsInBarrelGirthThreshold,
			minBarrelHeight);
	}
}

/* 
 * Draws slanted lines in the specified rectangle that reflect
 * the current value of the "taperTolerance" parameter.
 */
// private
void drawTaperToleranceForBarrel(Graphics& g, int x, int y, int width, int height) {
	int taperInset = (int) (width * (1.0 - taperTolerance))/2;
	g.drawLine(
		x + taperInset,
		y,
		x,
		y + height - 1);
	g.drawLine(
		x + width - 1 - taperInset,
		y,
		x + width - 1,
		y + height - 1);
}

// private
int countNonignoredOrangePixels(int off, int len) {
	int numOrangePixelsInRow = 0;
	for (; len>0; len--, off++) {
		if (pixelIsOrange[off] && !pixelIsWithinBarrel[off]) {
			numOrangePixelsInRow++;
		}
	}
	return numOrangePixelsInRow;
}

// NOTE: uses the same algorithm as WhileHighlighterFilter for identifying white pixels
// private
int countNonignoredWhitePixels(int off, int len) {
	int numWhitePixelsInRow = 0;
	for (; len>0; len--, off++) {
		if (pixelIsWhite[off] && !pixelIsWithinBarrel[off]) {
			numWhitePixelsInRow++;
		}
	}
	return numWhitePixelsInRow;
}
