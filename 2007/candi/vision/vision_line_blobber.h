#ifndef _VISION_LINE_BLOBBER_H_
#define _VISION_LINE_BLOBBER_H_
/*This is Paul's crazy last minute line finding code*/

#include "Point2D.h"
#include "Buffer2D.h"
#include "Pixel.h"
#include "Line.h"

#define MAX_N_LINES 20
#define MIN_LINE_BLOB_POINTS 200
#define MAX_LINE_GAP_PIXELS 5
#define MAX_SCHLUNKING_DISTANCE (.017*pixelIsWhite.height) //see note 1
#define LINE_MARGIN (.017*2.5*pixelIsWhite.height)

/* Thread Safe to Read */	
extern Line<int> whitelines[];		//array of the white lines we've found so far
extern int numwhitelines;			//number of lines so far in the line array

extern Line<int> inferredlines[];	//array of "lines" I suspect we shouldn't cross
extern int numinferredlines;		//number of inferred lines

extern Line<int> dashedlines[];		//lines where I think the dashed lines should connect
extern int numdashedlines;			//number of dashed lines

/* Not Thread Safe */
extern Buffer2D<Pixel> paulBlob;
extern Buffer2D<bool> whiteFilterMask;

void visBlobLines(void);

#endif //_VISION_LINE_BLOBBER_H_

/*	Notes
 *
 *	1. "Schlunking" is the process of connecting very close endpoints of
 *		lines to fill small gaps. It is named after the humorous sound that
 *		lines make when schlunked together. Not yet implemented.
 *		
 *		
 *		
 *		
 */
