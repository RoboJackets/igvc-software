#include "vision_util.h"

// ### MATH ###

int Math_min(int n1, int n2) { return (n1 < n2) ? n1 : n2; }
int Math_max(int n1, int n2) { return (n1 > n2) ? n1 : n2; }

float min3f(float f1, float f2, float f3) {
	if (f1 < f2) {
		return (f1 < f3) ? f1 : f3;
	} else {
		return (f2 < f3) ? f2 : f3;
	}
}

float max3f(float f1, float f2, float f3) {
	if (f1 > f2) {
		return (f1 > f3) ? f1 : f3;
	} else {
		return (f2 > f3) ? f2 : f3;
	}
}

int min3i(int i1, int i2, int i3) {
	if (i1 < i2) {
		return (i1 < i3) ? i1 : i3;
	} else {
		return (i2 < i3) ? i2 : i3;
	}
}

int max3i(int i1, int i2, int i3) {
	if (i1 > i2) {
		return (i1 > i3) ? i1 : i3;
	} else {
		return (i2 > i3) ? i2 : i3;
	}
}

// ### HSB ###

// This algorithm was taken from Wikipedia
// -> http://en.wikipedia.org/wiki/HSV_color_space
HSB RGBtoHSB(Pixel pixel) {
	int red = pixel.red;
	int green = pixel.green;
	int blue = pixel.blue;
	
	int max = Math_max(red, Math_max(green, blue));
	int min = Math_min(red, Math_min(green, blue));
	int range = max - min;
	
	int rawHue = -1;
	if (range != 0) {
		if (max == red)
			rawHue = 60*(green - blue)/(range) + 0;
		if (max == green)
			rawHue = 60*(blue - red)/(range) + 120;
		if (max == blue)
			rawHue = 60*(red - green)/(range) + 240;
	} else {
		// XXX: this is the case where the HUE becomes undefined
		rawHue = 0;
	}
	rawHue %= 360;
	
	int hue = rawHue * 256/360;
	int saturation = (max != 0)
		? (range)*255/max
		: 0;	// XXX: this is the case where the SATURATION becomes undefined
	int brightness = max;
	
	HSB result;
	result.hue = hue;
	result.saturation = saturation;
	result.brightness = brightness;
	return result;
}

// PERF: WARNING: This algorithm has not been optimized for speed (and is very slow at present).
// This algorithm was taken from Wikipedia
// -> http://en.wikipedia.org/wiki/HSV_color_space
Pixel HSBtoRGB(HSB hsb) {
	int H = ((int) hsb.hue) * 360/256;
	double S = ((double) hsb.saturation) / 256;
	double V = ((double) hsb.brightness) / 256;
	
	if (hsb.saturation == 0) {
		return Pixel(hsb.brightness, hsb.brightness, hsb.brightness);
	} else {
		int H_i = (H / 60) % 6;
		double f = ((double) H) / 60 - ((double) H_i);
		
		double p = V * (1 - S);
		double q = V * (1 - f*S);
		double t = V * (1 - (1-f)*S);
		
		u8 V_out = hsb.brightness;
		u8 p_out = (u8) (p*255);
		u8 q_out = (u8) (q*255);
		u8 t_out = (u8) (t*255);
		
		switch (H_i) {
		case 0: return Pixel((u8) V_out, (u8) t_out, (u8) p_out);
		case 1: return Pixel((u8) q_out, (u8) V_out, (u8) p_out);
		case 2: return Pixel((u8) p_out, (u8) V_out, (u8) t_out);
		case 3: return Pixel((u8) p_out, (u8) q_out, (u8) V_out);
		case 4: return Pixel((u8) t_out, (u8) p_out, (u8) V_out);
		case 5: return Pixel((u8) V_out, (u8) p_out, (u8) q_out);
		default: return Pixel(0, 0, 0);		// this should never happen
		}
	}
}

float HSLHueToRGBChannel(float m1, float m2, float H);

const float FLOAT_1_3 = ((float) 1) / 3;
const float FLOAT_1_2 = ((float) 1) / 2;
const float FLOAT_2_3 = ((float) 2) / 3;

// PERF: WARNING: This algorithm has not been optimized for speed (and is very slow at present).
// This algorithm was taken from:
// -> http://www.w3.org/TR/css3-color/#hsl-color
Pixel HSLtoRGB(HSL hsl) {
	float H = ((float) hsl.hue)/255;
	float S = ((float) hsl.saturation)/255;
	float L = ((float) hsl.lightness)/255;
	
	float m2 = (L <= FLOAT_1_2)
		? ( L*(S+1) )
		: ( L + S - L*S );
	
	float m1 = L*2 - m2;
	
	float R = HSLHueToRGBChannel( m1, m2, H + FLOAT_1_3 );
	float G = HSLHueToRGBChannel( m1, m2, H );
	float B = HSLHueToRGBChannel( m1, m2, H - FLOAT_1_3 );
	return Pixel(
		(u8) (255*R),
		(u8) (255*G),
		(u8) (255*B));
}

//private
float HSLHueToRGBChannel(float m1, float m2, float H) {
	// mod to 0.0 to 1.0 range
	if (H < 0.0) H += 1.0;
	if (H > 1.0) H -= 1.0;
	
	if (H*6 < 1) return m1 + (m2-m1)*H*6;
	if (H*2 < 1) return m2;
	if (H*3 < 2) return m1 + (m2-m1)*(FLOAT_2_3-H)*6;
	return m1;
}

// This algorithm was taken from:
// -> http://www.easyrgb.com/math.php?MATH=M18#text18
HSL RGBtoHSL_vSlow(Pixel rgb) {
	float R = rgb.red;
	float G = rgb.green;
	float B = rgb.blue;
	
	float var_R = ( R / 255 );                     //Where RGB values = 0 ÷ 255
	float var_G = ( G / 255 );
	float var_B = ( B / 255 );
	
	float var_Min = min3f( var_R, var_G, var_B );   //Min. value of RGB
	float var_Max = max3f( var_R, var_G, var_B );   //Max. value of RGB
	float del_Max = var_Max - var_Min;            //Delta RGB value
	
	float L = ( var_Max + var_Min ) / 2;
	
	float H, S;
	if ( del_Max == 0 )                     //This is a gray, no chroma...
	{
		H = 0;                                //HSL results = 0 ÷ 1
		S = 0;
	}
	else                                    //Chromatic data...
	{
		if ( L < 0.5 ) S = del_Max / ( var_Max + var_Min );
		else           S = del_Max / ( 2 - var_Max - var_Min );
		
		float del_R = ( ( ( var_Max - var_R ) / 6 ) + ( del_Max / 2 ) ) / del_Max;
		float del_G = ( ( ( var_Max - var_G ) / 6 ) + ( del_Max / 2 ) ) / del_Max;
		float del_B = ( ( ( var_Max - var_B ) / 6 ) + ( del_Max / 2 ) ) / del_Max;
		
		if      ( var_R == var_Max ) H = del_B - del_G;
		else if ( var_G == var_Max ) H = ( FLOAT_1_3 ) + del_R - del_B;
		else if ( var_B == var_Max ) H = ( FLOAT_2_3 ) + del_G - del_R;
		
		if ( H < 0 ) H += 1;
		if ( H > 1 ) H -= 1;
	}
	
	u8 norm_H = (u8) (H * 255);
	u8 norm_S = (u8) (S * 255);
	u8 norm_L = (u8) (L * 255);
	return HSL(norm_H, norm_S, norm_L);
}

// This algorithm was adapted from:
// -> http://www.easyrgb.com/math.php?MATH=M18#text18
HSL RGBtoHSL(Pixel rgb) {
	int var_R = rgb.red;                     //Where RGB values = 0 ÷ 255
	int var_G = rgb.green;
	int var_B = rgb.blue;
	
	int var_Min = min3i( var_R, var_G, var_B );   //Min. value of RGB
	int var_Max = max3i( var_R, var_G, var_B );   //Max. value of RGB
	int del_Max = var_Max - var_Min;            //Delta RGB value
	
	int L = ( var_Max + var_Min ) / 2;
	
	int H, S;
	if ( del_Max == 0 )                     //This is a gray, no chroma...
	{
		H = 0;                                //HSL results = 0 ÷ 1
		S = 0;
	}
	else                                    //Chromatic data...
	{
		if ( L < 128 ) S = 255 * del_Max / ( var_Max + var_Min );
		else           S = 255 * del_Max / ( (255 * 2) - ( var_Max + var_Min ) );
		
		// NOTE: The numerical correctness of this could probably be improved
		//       by rewriting this expression to defer dividing until the end
		int del_R = 255 * ( ( ( var_Max - var_R ) / 6 ) + ( del_Max / 2 ) ) / del_Max;
		int del_G = 255 * ( ( ( var_Max - var_G ) / 6 ) + ( del_Max / 2 ) ) / del_Max;
		int del_B = 255 * ( ( ( var_Max - var_B ) / 6 ) + ( del_Max / 2 ) ) / del_Max;
		
		if      ( var_R == var_Max ) H = del_B - del_G;
		else if ( var_G == var_Max ) H = ( 255 * 1/3 ) + del_R - del_B;
		else if ( var_B == var_Max ) H = ( 255 * 2/3 ) + del_G - del_R;
		
		if ( H < 0 )   H += 255;
		if ( H > 255 ) H -= 255;
	}
	
	return HSL((u8) H, (u8) S, (u8) L);
}
