#ifndef _BUFFER_2D_H_
#define _BUFFER_2D_H_

#include "types.h"	// for NULL
#include <string.h>		// for memcpy

/*
 * Utility class that represents a resizable 2D matrix of elements.
 */
template<class E>
class Buffer2D
{
//private:
public:
	E* data;
	int width;
	int height;
	
	// ### INIT ###
public:
	Buffer2D<E>() {
		this->init(0, 0);
	}
	
	Buffer2D<E>(int width, int height) {
		this->init(width, height);
	}
	
private:
	void init(int width, int height) {
		this->data = NULL;
		this->width = 0;
		this->height = 0;
		
		this->resize(width, height);
	}
	
public:
//	~Buffer2D<E>() {
		// Make sure the data buffer (if one existed) is deallocated
//		this->resize(0,0);
//	}
	
	// ### ACCESSORS ###
public:
	//E* data() { return this->data; }
	//int width() { return this->width; }
	//int height() { return this->height; }
	int numElements() { return this->width * this->height; }
	
	// Returns the element at the specified location.
	// (The location is NOT bound-checked.)
	E get(int x, int y) {
		return *(this->at(x,y));
	}
	
	// Changes the element at the specified location.
	// (The location is NOT bound-checked.)
	void set(int x, int y, E newElement) {
		*(this->at(x,y)) = newElement;
	}
	
	E& operator[](int index) {
		return this->data[index];
	}
	
	E& at(int x, int y) {
		return this->data[y*width + x];
	}
	
	// Returns the address of the first (leftmost) element on the specified row.
	// (The row index is NOT bound-checked.)
	E* atRow(int y);
	
	// ### OPERATIONS ###
	
	// Resizes this buffer (if the specified new size is different than the old size).
	// If width or height is 0, then no new buffer will be allocated.
	// If the resize is successful and a new buffer is allocated,
	// the contents of the new buffer are undefined.
	// 
	// Returns TRUE if the buffer was resized, or
	//         FALSE if the new size was the same as the old size.
	bool resize(int width, int height) {
		// Abort if the new size is the same as the old size
		if ((this->width == width) && (this->height == height)) {
			return FALSE;
		}
		
		// Update width and height
		this->width = width;
		this->height = height;
		
		// Free old data buffer (if one was allocated)
		if (this->data != NULL) {
			delete[] this->data;
		}
		
		// Allocate new data buffer (if width & height are non-zero)
		if ((width != 0) && (height != 0)) {
			this->data = new E[width * height];
		} else {
			this->data = NULL;
		}
		
		return TRUE;
	}
	
	// Resizes this buffer to be the same size as the specified buffer
	// (if it wasn't already the same size).
	// 
	// Returns TRUE if the buffer was resized, or
	//         FALSE if the new size was the same as the old size.
	template<class F>
	bool resizeToMatch(Buffer2D<F>& buffer) {
		return this->resize(buffer.width, buffer.height);
	}
	
	// Copies the data from the specified buffer, resizing if necessary.
	// 
	// Returns TRUE if the buffer was resized, or
	//         FALSE if the new size was the same as the old size.
	bool copyFrom(Buffer2D<E>& buffer) {
		return this->copyFrom(buffer.width, buffer.height, buffer.data);
	}
	bool copyFrom(int width, int height, E* data) {
		bool didResize = this->resize(width, height);
		memcpy(this->data, data, sizeof(E) * this->numElements());
		return didResize;
	}
};

#endif
