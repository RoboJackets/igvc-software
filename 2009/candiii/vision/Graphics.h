/*
 * This class emulates Java's Graphics class.
 *
 * The class performs all graphics operations on the
 * IplImage that it is passed upon construction.
 */

#ifndef _GRAPHICS_H_
#define _GRAPHICS_H_

#include "image_buffers.h"
#include "Point2D.h"
#include <stdlib.h>			// abs
#include <QVector>


//int Math_min(int n1, int n2)
//{
//	return (n1 < n2) ? n1 : n2;
//}
//int Math_max(int n1, int n2)
//{
//	return (n1 > n2) ? n1 : n2;
//}



class Graphics
{
private:
	IplImage* buffer;
	CvScalar color;

public:
	Graphics(IplImage* buffer)
	{
		this->buffer = buffer;
		color = CV_RGB(0,0,0); //default to black
	}

static int Math_min(int n1, int n2)
{
	return (n1 < n2) ? n1 : n2;
}
static int Math_max(int n1, int n2)
{
	return (n1 > n2) ? n1 : n2;
}

	void setColor(const CvScalar color)
	{
		this->color = color;
	}

	void drawRect(int x, int y, int width, int height)
	{
		//int xMax = x + width;
		//int yMax = y + height;
		cvRectangle( buffer, cvPoint(x,y), cvPoint(x+width, y+height), color, 1, 8, 0 );
	}

	void drawRect_rational(int x, int y, int width, int height)
	{
		drawRect(x, y, width-1, height-1);
	}

	void fillRect_rational(int x, int y, int width, int height)
	{
		//int xMax = x + width;
		//int yMax = y + height;
		cvRectangle( buffer, cvPoint(x,y), cvPoint(x+width, y+height), color, CV_FILLED, 8, 0 );
	}

	void drawHorizLine(int y, int x1, int x2)
	{
		cvLine( buffer, cvPoint(x1,y), cvPoint(x2,y), color, 1, 8, 0 );
	}

	void drawVertLine(int x, int y1, int y2)
	{
		cvLine( buffer, cvPoint(x,y1), cvPoint(x,y2), color, 1, 8, 0 );
	}

	void drawLine(int x1, int y1, int x2, int y2)
	{
		cvLine( buffer, cvPoint(x1,y1), cvPoint(x2,y2), color, 1, 8, 0 );
	}

	void drawPixel(int x, int y)
	{
		cvLine( buffer, cvPoint(x,y), cvPoint(x,y), color, 1, 8, 0 );
	}

//	template<class N>
//	void drawLine(Line<N> L) {
//		int x1=L.a.x;	int y1=L.a.y;
//		int x2=L.b.x;	int y2=L.b.y;
//
//		drawLine(x1, y1, x2, y2);
//	}

	// Calculates the points that would be used in drawing a line
	// from (x1, y1) to (x2, y2) and appends them to 'outLinePixels'.
	static void calculatePointsInLine(int x1, int y1, int x2, int y2, QVector< Point2D<int> >* outLinePixels)
	{
		int deltaX = x2 - x1;
		int deltaY = y2 - y1;
		int stepMax = Math_max(abs(deltaX), abs(deltaY)) + 1;
		for (int i=0; i<=stepMax; i++)
		{
			outLinePixels->append(Point2D<int>(
									  x1 + (deltaX * i/stepMax),
									  (y1 + (deltaY * i/stepMax))));
		}
	}

	// Returns whether the specified point is in the bounds of this graphics context
	bool contains(int x, int y)
	{
		if ((x < 0) || (x >= buffer->width)) return false;
		if ((y < 0) || (y >= buffer->height)) return false;
		return true;
	}

	// ### ACCESSORS ###

	int width()
	{
		return this->buffer->width;
	}

	int height()
	{
		return this->buffer->height;
	}
};

#endif
