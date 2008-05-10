#ifndef _PIXELRGB_H_
#define _PIXELRGB_H_

#include "types.h"
namespace blackbarrels{

//Data type for pixel data. Note: this should only be used for RGB data
//Paul's user friendly and c friendly version
typedef struct _PixelRGB {
	u8 r;
	u8 g;
	u8 b;

	_PixelRGB() { /* empty */ }

	_PixelRGB (u8 r, u8 g, u8 b) {
		this->r = r;
		this->g = g;
		this->b = b;
	}

	// 0 <= curStep <= maxStep
	static _PixelRGB fadeBetween (_PixelRGB srcColor, _PixelRGB dstColor, int curStep, int maxStep) {
		return _PixelRGB (
		           fadeBetween (srcColor.r, dstColor.r, curStep, maxStep),
		           fadeBetween (srcColor.g, dstColor.g, curStep, maxStep),
		           fadeBetween (srcColor.b, dstColor.b, curStep, maxStep));
	}

//private:
	// 0 <= curStep <= maxStep
	static u8 fadeBetween (u8 srcChannel, u8 dstChannel, int curStep, int maxStep) {
		signed int srcChannel_sint = (signed int) srcChannel;
		signed int dstChannel_sint = (signed int) dstChannel;
		signed int curStep_sint = (signed int) curStep;
		signed int maxStep_sint = (signed int) maxStep;

		signed int deltaChannel_sint = dstChannel_sint - srcChannel_sint;
		signed int fadeOffset_sint = deltaChannel_sint * curStep_sint / maxStep_sint;
		return (u8) (srcChannel_sint + fadeOffset_sint);
	}
	_PixelRGB operator& (_PixelRGB p2) {
		_PixelRGB out(this->r& p2.r,this->g& p2.g,this->b& p2.b);
			return out;
		}
} PixelRGB;
}
#endif // _PIXEL_H_
