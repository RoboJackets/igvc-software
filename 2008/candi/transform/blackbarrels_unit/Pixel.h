#ifndef _PIXEL_H_
#define _PIXEL_H_

#include "types.h"
namespace blackbarrels{

//Data type for pixel data. Note: this should only be used for RGB data
typedef struct _Pixel {
	u8 red;
	u8 green;
	u8 blue;

	_Pixel() { /* empty */ }

	_Pixel (u8 red, u8 green, u8 blue) {
		this->red = red;
		this->green = green;
		this->blue = blue;
	}

	// 0 <= curStep <= maxStep
	static _Pixel fadeBetween (_Pixel srcColor, _Pixel dstColor, int curStep, int maxStep) {
		return _Pixel (
		           fadeBetween (srcColor.red, dstColor.red, curStep, maxStep),
		           fadeBetween (srcColor.green, dstColor.green, curStep, maxStep),
		           fadeBetween (srcColor.blue, dstColor.blue, curStep, maxStep));
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
} Pixel;
}
#endif // _PIXEL_H_
