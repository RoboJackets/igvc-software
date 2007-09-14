/*
 * This class emulates Java's Graphics class.
 * 
 * The class performs all graphics operations on the
 * Buffer2D<Pixel> that it is passed upon construction.
 */

#ifndef _GRAPHICS_H_
#define _GRAPHICS_H_

#include "Pixel.h"
#include "Buffer2D.h"
#include "Point2D.h"
#include "vision_util.h"
#include "Line.h"
#include <stdlib.h>			// abs
#include <QVector>

class Graphics
{
private:
	Buffer2D<Pixel>* buffer;
	Pixel color;
	
public:
	Graphics(Buffer2D<Pixel>* buffer) {
		this->buffer = buffer;
		this->color = Pixel(0, 0, 0);	// black
	}
	
	// ### STATE ###
	
	void setColor(const Pixel& color) {
		this->color = color;
	}
	
	// ### DRAWING ###
	
	void drawRect(int x, int y, int widthMinusOne, int heightMinusOne) {
		int xMax = x + widthMinusOne;
		int yMax = y + heightMinusOne;
		
		// Draw top edge
		drawHorizLine(y, x, xMax);
		// Draw bottom edge
		drawHorizLine(yMax, x, xMax);
		
		// Draw left edge
		drawVertLine(x, y, yMax);
		// Draw right edge
		drawVertLine(xMax, y, yMax);
	}
	
	void drawRect_rational(int x, int y, int width, int height) {
		drawRect(x, y, width-1, height-1);
	}
	
	void fillRect_rational(int x, int y, int width, int height) {
		if (width <= 0) return;
		
		int xMax = x+width-1;
		for (int i=height; i>0; i--, y++) {
			drawHorizLine(y, x, xMax);
		}
	}
	
	void drawHorizLine(int y, int x1, int x2) {
		// Check bounds and limit if necessary
		if ((y < 0) || (y >= buffer->height)) return;
		if (x1 < 0) x1 = 0;
		if (x2 >= buffer->width) x2 = buffer->width-1;
		
		for (int curX=x1; curX<=x2; curX++) {
			buffer->set(curX, y, this->color);
		}
	}

	void drawVertLine(int x, int y1, int y2) {
		// Check bounds and limit if necessary
		if ((x < 0) || (x >= buffer->width)) return;
		if (y1 < 0) y1 = 0;
		if (y2 >= buffer->height) y2 = buffer->height-1;
		
		for (int curY=y1; curY<=y2; curY++) {
			buffer->set(x, curY, this->color);
		}
	}
	
	void drawPixel(int x, int y) {
		// Check bounds
		if (!this->contains(x, y)) return;
		
		buffer->set(x, y, this->color);
	}
	
	void drawLine(int x1, int y1, int x2, int y2) {
		int deltaX = x2-x1;
		int deltaY = y2-y1;
		
		int stepMax = Math_max(abs(deltaX), abs(deltaY)) + 1;
		for (int i=0; i<=stepMax; i++) {
			drawPixel(
				x1 + (deltaX * i/stepMax),
				y1 + (deltaY * i/stepMax));
		}
	}
	
	template<class N>
	void drawLine(Line<N> L) {
		int x1=L.a.x;	int y1=L.a.y;
		int x2=L.b.x;	int y2=L.b.y;
		
		drawLine(x1, y1, x2, y2);
	}
	
	// Calculates the points that would be used in drawing a line
	// from (x1, y1) to (x2, y2) and appends them to 'outLinePixels'.
	static void calculatePointsInLine(int x1, int y1, int x2, int y2, QVector< Point2D<int> >* outLinePixels) {
		int deltaX = x2-x1;
		int deltaY = y2-y1;
		
		int stepMax = Math_max(abs(deltaX), abs(deltaY)) + 1;
		for (int i=0; i<=stepMax; i++) {
			outLinePixels->append(Point2D<int>(
				x1 + (deltaX * i/stepMax),
				y1 + (deltaY * i/stepMax)));
		}
	}
	
	// Returns whether the specified point is in the bounds of this graphics context
	bool contains(int x, int y) {
		if ((x < 0) || (x >= buffer->width)) return false;
		if ((y < 0) || (y >= buffer->height)) return false;
		return true;
	}
	
	// ### ACCESSORS ###
	
	int width() {
		return this->buffer->width;
	}
	
	int height() {
		return this->buffer->height;
	}
};

#endif
