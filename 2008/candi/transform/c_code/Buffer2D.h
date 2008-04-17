#ifndef _BUFFER_2D_H_
#define _BUFFER_2D_H_

#include "types.h"	// for NULL
#include <string.h>		// for memcpy
#include "PixelRGB.h"
#include <stdlib.h>

/*
 * Utility class that represents a resizable 2D matrix of elements.
 * Contains specialization for PixelRGB form.
 */
 
/* Signature:
 * 	template<class E>
 *	class Buffer2D {
 
 	public:
 		E* data;
		int width;
		int height;
	### CONSTRUCTORS ###
		Buffer2D<E>();
		Buffer2D<E> (int width, int height);
	### FUNCTIONS ###
	int 			numElements		();
	E 				get 			(int x, int y) ;
	void			set 			(int x, int y, E newElement) ;
	E& 				operator[] 		(int index) ;
	E& 				at 				(int x, int y) ;
	bool 			resize 			(int width, int height) ;
	bool 			resizeToMatch 	(Buffer2D<F>& buffer) ;
	bool 			copyFrom 		(Buffer2D<E>& buffer) ;
	bool 			copyFrom 		(int width, int height, E* data) ;
	Buffer2D<E>& 	getLine			(int y)

*/
template<class E>
class Buffer2D {
//private:
	public:
		E* data;
		int width;
		int height;

		// ### INIT ###
	public:
		Buffer2D<E>() {
			this->init (0, 0);
		}

		Buffer2D<E> (int width, int height) {
			this->init (width, height);
		}

	private:
		void init (int width, int height) {
			this->data = NULL;
			this->width = 0;
			this->height = 0;

			this->resize (width, height);
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
		// Paul: I suspect this and set may break if used
		E get (int x, int y) {
			return * (this->at (x,y));
		}

		// Changes the element at the specified location.
		// (The location is NOT bound-checked.)
		void set (int x, int y, E newElement) {
			* (this->at (x,y)) = newElement;
		}

		E& operator[] (int index) {
			return this->data[index];
		}

		E& at (int x, int y) {
			return this->data[y*width + x];
		}

		// Returns the address of the first (leftmost) element on the specified row.
		// (The row index is NOT bound-checked.)
		E* atRow (int y);

		// ### OPERATIONS ###

		// Resizes this buffer (if the specified new size is different than the old size).
		// If width or height is 0, then no new buffer will be allocated.
		// If the resize is successful and a new buffer is allocated,
		// the contents of the new buffer are undefined.
		//
		// Returns TRUE if the buffer was resized, or
		//         FALSE if the new size was the same as the old size.
		bool resize (int width, int height) {
			// Abort if the new size is the same as the old size
			if ( (this->width == width) && (this->height == height)) {
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
			if ( (width != 0) && (height != 0)) {
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
		bool resizeToMatch (Buffer2D<F>& buffer) {
			return this->resize (buffer.width, buffer.height);
		}
		
		/** Shrinks this image in place by a factor of <tt>f</tt>. */
		bool shrink(int f) {
			if ( f==1) {
				return FALSE;
			}
			
			int w,h,wn,hn,x,yc,xn,ync,hl,ycinc,pick;
			E* p=this->data;
			w=this->width;
			h=this->height;
			wn=w/f;
			hn=h/f;
			hl=h*w;
			ycinc=f*w;
			pick=f/2;
			
			if ( (wn == 0) || (hn == 0)) {
				this->resize(wn,hn);	// will delete data buffer
				return TRUE;
			}
			
			for(yc=pick*w,ync=0; yc<hl; yc+=ycinc,ync+=wn){
				for(x=pick,xn=0; x<w; x+=f,xn++){
					p[xn+ync]=p[x+yc];
				}
			}
						
			this->width = wn;
			this->height = hn;
			return TRUE;
		}
		
		
		bool grow(int f) {
			E* p;
			E* pn;
			int rowsize1,rowsize2,n1,n2,numel1,i,roff1,roff2,forwardroff,next_segment_row_off;
			rowsize1=this->width;
			numel1=this->numElements();
			rowsize2=f*rowsize1;
			p= (this->data);
			pn=new E[numel1*f*f];
			
			
			
			for(roff1=roff2=0;roff1<numel1;){
				for(n1=0,n2=0;n2<rowsize2;n1++){
					for(i=0;i<f;i++,n2++){
						pn[roff2+n2]=p[roff1+n1];
					}
				}
				next_segment_row_off=roff2+rowsize2*f;
				for(forwardroff=roff2+rowsize2; 
					forwardroff<next_segment_row_off;
					forwardroff+=rowsize2){
						memcpy((void*)&(pn[forwardroff]),(void*)&(pn[roff2]),rowsize2*sizeof(E));
					}
				roff1+=rowsize1;
				roff2=next_segment_row_off;
			}
			this->data=pn;
			this->width=rowsize2;
			this->height=this->height*f;
			delete []p;
			return TRUE;
		}
		
		
		// Shrinks this buffer and its contents (if the specified new size is
		// different than the old size).
		// If width or height is 0, then no new buffer will be allocated.
		// If the shrink is successful and a new buffer is allocated,
		// the contents of the new buffer are (roughly) the nearest neighbor
		// interpolation of the contents of the original .
		//
		// Returns TRUE if the buffer was shrunk, or
		//         FALSE if the new size was the same as the old size.
		// DEPRICATED!
		bool shrinkto (int width, int height) {
			// Abort if the new size is the same as the old size
			if ( (this->width == width) && (this->height == height)) {
				return FALSE;
			}
			E* p,pn;
			// Allocate new data buffer (if width & height are non-zero)
			if ( (width != 0) && (height != 0)) {
				p  = this->data;
				//pn = new E[width * height];
			} else {
				// Free old data buffer (if one was allocated)
				if (this->data != NULL) {
					delete[] this->data;
				}
				this->data = NULL;
				return TRUE;
			}
			int w,h,wn,hn,hl,xinc,yinc,ync,yc,xn,x;
			w=this->width;
			h=this->height;
			wn=width;
			hn=height;
			hl=(hn-1)*wn;
			xinc=w/wn;
			yinc=h/hn*w;
			
			for(ync=0, yc=0; ync<=hl; ync+=wn, yc+=yinc){
				for(xn=0, x=0; xn<wn; xn++, x+=xinc){
					p[ync+xn]=p[yc+x];
				}
			}
			//delete[] p;
			//this.data=pn;
			
			
			
			
			
			// Update width and height to new ones
			this->width = width;
			this->height = height;
			return TRUE;
		}
		
		
		

		// Copies the data from the specified buffer, resizing if necessary.
		//
		// Returns TRUE if the buffer was resized, or
		//         FALSE if the new size was the same as the old size.
		bool copyFrom (Buffer2D<E>& buffer) {
			return this->copyFrom (buffer.width, buffer.height, buffer.data);
		}
		
		bool copyFrom (int width, int height, E* data) {
			bool didResize = this->resize (width, height);
			memcpy (this->data, data, sizeof (E) * this->numElements());
			return didResize;
		}
		
		//returns an alias to the line of this image
		Buffer2D<E>* getLine(int y){
			Buffer2D<E>* line=new Buffer2D<E>;
			
			line->resize(this->width,1);
			delete[] (line->data);//free the useless line we just allocated
			line->data=&this->at(0,y);
			return line;
		}
		
		// ### SPECIALIZED ###
		Buffer2D<PixelRGB> toRGB(){
			int ii;
			int buffLength = this->numElements();
			Buffer2D<E> img=*this;
			Buffer2D<PixelRGB> dst;
			dst.resizeToMatch(img);
			//standard reverse boolean conversion
			for(ii = 0; ii < buffLength; ii++){
				if(img[ii])
					dst[ii].r = dst[ii].g = dst[ii].b = 255;
				else
					dst[ii].r = dst[ii].g = dst[ii].b = 0;
			}	
			Buffer2D<PixelRGB>* out=new Buffer2D<PixelRGB>;
			*out=dst;
			return dst;
		}
		
		Buffer2D<bool> toBool(){
			int ii;
			int buffLength = this->numElements();
			Buffer2D<E> img=*this;
			Buffer2D<bool> dst;
			dst.resizeToMatch(img);
			//standard boolean conversion
			for(ii = 0; ii < buffLength; ii++){
				if(img[ii].r > 0 || img[ii].g > 0 || img[ii].b > 0)
					dst[ii] = 1;
				else
					dst[ii] = 0;
			}
			Buffer2D<bool>* out=new Buffer2D<bool>;
			*out=dst;
			return dst;
		}
		
};





















/*
template<>
class Buffer2D <PixelRGB>{
//private:
	public:
		PixelRGB* data;
		int width;
		int height;

		// ### INIT ###
	public:
		Buffer2D<PixelRGB>() {
			this->init (0, 0);
		}

		Buffer2D<PixelRGB> (int width, int height) {
			this->init (width, height);
		}

	private:
		void init (int width, int height) {
			this->data = NULL;
			this->width = 0;
			this->height = 0;

			this->resize (width, height);
		}

	public:
//	~Buffer2D<PixelRGB>() {
		// Make sure the data buffer (if one existed) is deallocated
//		this->resize(0,0);
//	}

		// ### ACCESSORS ###
	public:
		//PixelRGB* data() { return this->data; }
		//int width() { return this->width; }
		//int height() { return this->height; }
		int numElements() { return this->width * this->height; }

		// Returns the element at the specified location.
		// (The location is NOT bound-checked.)
		PixelRGB get (int x, int y) {
			return (this->at (x,y));
		}

		// Changes the element at the specified location.
		// (The location is NOT bound-checked.)
		void set (int x, int y, PixelRGB newElement) {
			(this->at (x,y)) = newElement;
		}

		PixelRGB& operator[] (int index) {
			return this->data[index];
		}

		PixelRGB& at (int x, int y) {
			return this->data[y*width + x];
		}

		// Returns the address of the first (leftmost) element on the specified row.
		// (The row index is NOT bound-checked.)
		PixelRGB* atRow (int y);

		// ### OPERATIONS ###

		// Resizes this buffer (if the specified new size is different than the old size).
		// If width or height is 0, then no new buffer will be allocated.
		// If the resize is successful and a new buffer is allocated,
		// the contents of the new buffer are undefined.
		//
		// Returns TRUE if the buffer was resized, or
		//         FALSE if the new size was the same as the old size.
		bool resize (int width, int height) {
			// Abort if the new size is the same as the old size
			if ( (this->width == width) && (this->height == height)) {
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
			if ( (width != 0) && (height != 0)) {
				this->data = new PixelRGB[width * height];
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
		bool resizeToMatch (Buffer2D<F>& buffer) {
			return this->resize (buffer.width, buffer.height);
		}
		
		// Shrinks this buffer and its contents (if the specified new size is
		// different than the old size).
		// If width or height is 0, then no new buffer will be allocated.
		// If the shrink is successful and a new buffer is allocated,
		// the contents of the new buffer are (roughly) the nearest neighbor
		// interpolation of the contents of the original .
		//
		// Returns TRUE if the buffer was shrunk, or
		//         FALSE if the new size was the same as the old size.
		
		
		bool shrink (int width, int height) {
			// Abort if the new size is the same as the old size
			if ( (this->width == width) && (this->height == height)) {
				return FALSE;
			}
			PixelRGB* p;
			PixelRGB* pn;
			// Allocate new data buffer (if width & height are non-zero)
			if ( (width != 0) && (height != 0)) {
				p  = this->data;
				//pn = new PixelRGB[(width * height)];
			} else {
				// Free old data buffer (if one was allocated)
				if (this->data != NULL) {
					delete[] this->data;
				}
				this->data = NULL;
				return TRUE;
			}
			int w,h,wn,hn,hl,xinc,yinc,ync,yc,xn,x;
			w=this->width;
			h=this->height;
			wn=width;
			hn=height;
			hl=(hn-1)*wn;
			xinc=w/wn;
			yinc=h/hn*w;
			
			for(ync=0, yc=0; ync<=hl; ync+=wn, yc+=yinc){
				for(xn=0, x=0; xn<wn; xn++, x+=xinc){
					p[ync+xn]=p[yc+x];
				}
			}
			//delete[] p;
			//this->data=pn;
			// Update width and height to new ones
			this->width = width;
			this->height = height;
			return TRUE;
		}
		
		
		
		// Copies the data from the specified buffer, resizing if necessary.
		//
		// Returns TRUE if the buffer was resized, or
		//         FALSE if the new size was the same as the old size.
		bool copyFrom (Buffer2D<PixelRGB>& buffer) {
			return this->copyFrom (buffer.width, buffer.height, buffer.data);
		}
		bool copyFrom (int width, int height, PixelRGB* data) {
			bool didResize = this->resize (width, height);
			memcpy (this->data, data, sizeof (PixelRGB) * this->numElements());
			return didResize;
		}
		
		//returns an alias to the line of this image
		Buffer2D<PixelRGB>* getLine(int y){
			Buffer2D<PixelRGB>* line=new Buffer2D<PixelRGB>;
			
			line->resize(this->width,1);
			delete (line->data);//free the useless line we just allocated
			line->data=&this->at(0,y);
			return line;
		}
		
		// ### SPECIALIZED ###
		Buffer2D<bool> toBool(){
			int ii;
			int buffLength = this->numElements();
			Buffer2D<PixelRGB>& img=*this;
			Buffer2D<bool>& dst=*(new Buffer2D<bool>);
			dst.resizeToMatch(img);
			/*standard boolean conversion/
			for(ii = 0; ii < buffLength; ii++){
				if(img[ii].r > 0 || img[ii].g > 0 || img[ii].b > 0)
					dst[ii] = 1;
				else
					dst[ii] = 0;
			}
			return dst;
		}
};*/














/*template<>
class Buffer2D <bool>{
//private:
	public:
		bool* data;
		int width;
		int height;

		// ### INIT ###
	public:
		Buffer2D<bool>() {
			this->init (0, 0);
		}

		Buffer2D<bool> (int width, int height) {
			this->init (width, height);
		}

	private:
		void init (int width, int height) {
			this->data = NULL;
			this->width = 0;
			this->height = 0;

			this->resize (width, height);
		}

	public:
//	~Buffer2D<bool>() {
		// Make sure the data buffer (if one existed) is deallocated
//		this->resize(0,0);
//	}

		// ### ACCESSORS ###
	public:
		//bool* data() { return this->data; }
		//int width() { return this->width; }
		//int height() { return this->height; }
		int numElements() { return this->width * this->height; }

		// Returns the element at the specified location.
		// (The location is NOT bound-checked.)
		bool get (int x, int y) {
			return (this->at (x,y));
		}

		// Changes the element at the specified location.
		// (The location is NOT bound-checked.)
		void set (int x, int y, bool newElement) {
			(this->at (x,y)) = newElement;
		}

		bool& operator[] (int index) {
			return this->data[index];
		}

		bool& at (int x, int y) {
			return this->data[y*width + x];
		}

		// Returns the address of the first (leftmost) element on the specified row.
		// (The row index is NOT bound-checked.)
		bool* atRow (int y);

		// ### OPERATIONS ###

		// Resizes this buffer (if the specified new size is different than the old size).
		// If width or height is 0, then no new buffer will be allocated.
		// If the resize is successful and a new buffer is allocated,
		// the contents of the new buffer are undefined.
		//
		// Returns TRUE if the buffer was resized, or
		//         FALSE if the new size was the same as the old size.
		bool resize (int width, int height) {
			// Abort if the new size is the same as the old size
			if ( (this->width == width) && (this->height == height)) {
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
			if ( (width != 0) && (height != 0)) {
				this->data = new bool[width * height];
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
		bool resizeToMatch (Buffer2D<F>& buffer) {
			return this->resize (buffer.width, buffer.height);
		}
		
		// Shrinks this buffer and its contents (if the specified new size is
		// different than the old size).
		// If width or height is 0, then no new buffer will be allocated.
		// If the shrink is successful and a new buffer is allocated,
		// the contents of the new buffer are (roughly) the nearest neighbor
		// interpolation of the contents of the original .
		//
		// Returns TRUE if the buffer was shrunk, or
		//         FALSE if the new size was the same as the old size.
		
		
		bool shrink (int width, int height) {
			// Abort if the new size is the same as the old size
			if ( (this->width == width) && (this->height == height)) {
				return FALSE;
			}
			bool* p;
			bool* pn;
			// Allocate new data buffer (if width & height are non-zero)
			if ( (width != 0) && (height != 0)) {
				p  = this->data;
				//pn = new bool[(width * height)];
			} else {
				// Free old data buffer (if one was allocated)
				if (this->data != NULL) {
					delete[] this->data;
				}
				this->data = NULL;
				return TRUE;
			}
			int w,h,wn,hn,hl,xinc,yinc,ync,yc,xn,x;
			w=this->width;
			h=this->height;
			wn=width;
			hn=height;
			hl=(hn-1)*wn;
			xinc=w/wn;
			yinc=h/hn*w;
			
			for(ync=0, yc=0; ync<=hl; ync+=wn, yc+=yinc){
				for(xn=0, x=0; xn<wn; xn++, x+=xinc){
					p[ync+xn]=p[yc+x];
				}
			}
			//delete[] p;
			//this->data=pn;
			// Update width and height to new ones
			this->width = width;
			this->height = height;
			return TRUE;
		}
		
		
		
		// Copies the data from the specified buffer, resizing if necessary.
		//
		// Returns TRUE if the buffer was resized, or
		//         FALSE if the new size was the same as the old size.
		bool copyFrom (Buffer2D<bool>& buffer) {
			return this->copyFrom (buffer.width, buffer.height, buffer.data);
		}
		bool copyFrom (int width, int height, bool* data) {
			bool didResize = this->resize (width, height);
			memcpy (this->data, data, sizeof (bool) * this->numElements());
			return didResize;
		}
		
		//returns an alias to the line of this image
		Buffer2D<bool>* getLine(int y){
			Buffer2D<bool>* line=new Buffer2D<bool>;
			
			line->resize(this->width,1);
			delete (line->data);//free the useless line we just allocated
			line->data=&this->at(0,y);
			return line;
		}
		
		// ### SPECIALIZED ###
		Buffer2D<bool> toRGB(){
			int ii;
			int buffLength = this->numElements();
			Buffer2D<bool>& img=*this;
			Buffer2D<PixelRGB>& dst=*(new Buffer2D<PixelRGB>);
			dst.resizeToMatch(img);
			/*standard reverse boolean conversion/
			for(ii = 0; ii < buffLength; ii++){
				if(img[ii])
					dst[ii].r = dst[ii].g = dst[ii].b = 255;
				else
					dst[ii].r = dst[ii].g = dst[ii].b = 0;
			}	
			return dst;
		}
};*/



#endif
