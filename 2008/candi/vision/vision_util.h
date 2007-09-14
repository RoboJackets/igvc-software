#ifndef _VISION_UTIL_H_
#define _VISION_UTIL_H_

#include "Pixel.h"
#include "types.h"

// ### MATH ###

int Math_min(int n1, int n2);
int Math_max(int n1, int n2);

float min3f(float f1, float f2, float f3);
float max3f(float f1, float f2, float f3);

int min3i(int i1, int i2, int i3);
int max3i(int i1, int i2, int i3);

// ### HSB ###

struct HSB {
	u8 hue;         // 0=0 degrees, 256(???)=360 degrees¡
	u8 saturation;  // 0=0%, 255=100%
	u8 brightness;	// 0=0%, 255=100%
	
	HSB() {}
	HSB(u8 hue, u8 saturation, u8 brightness) {
		this->hue = hue;
		this->saturation = saturation;
		this->brightness = brightness;
	}
};

struct HSL {
	u8 hue;		// 0=0 degrees, 255=360 degrees
	u8 saturation;	// 0=0%, 255=100%
	u8 lightness;	// 0=0%, 255=100%
	
	HSL() {}
	HSL(u8 hue, u8 saturation, u8 lightness) {
		this->hue = hue;
		this->saturation = saturation;
		this->lightness = lightness;
	}
};

HSB RGBtoHSB(Pixel pixel);
Pixel HSBtoRGB(HSB hsb);
Pixel HSLtoRGB(HSL hsl);
HSL RGBtoHSL(Pixel rgb);

#endif // _VISION_UTIL_H_
