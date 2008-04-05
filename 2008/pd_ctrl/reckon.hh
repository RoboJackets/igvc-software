#ifndef RECKON_HH
#define RECKON_HH

//#include <cstdio>
#include <cmath>
#include "misc.hh"


#define b 1//width of wheels

typedef struct{
	double x_0;
	double y_0;
	double x_t;
	double y_t;

	double th_0;

	struct timeval t1, t2;

	double t;
	double lastdt;	
	
	double x,y,th;
} ddrn;

void deadReckon(ddrn * reckon, float v_left, float v_right){
	gettimeofday(&(reckon->t2), NULL);

	reckon->lastdt = calcdt(reckon->t1, reckon->t2);

	reckon->t += reckon->lastdt;
		
	float r = ( b*(v_right + v_left) ) / ( 2*(v_left - v_right) );

	reckon->th = reckon->th_0 + (v_left - v_right)*reckon->t / b;

	reckon->y = reckon->y_0 - ( r ) * ( cos( (v_left - v_right)*(reckon->t)/b + reckon->th_0 ) - cos(reckon->th_0) );
	reckon->x = reckon->x_0 + ( r ) * ( sin( (v_left - v_right)*(reckon->t)/b + reckon->th_0 ) - sin(reckon->th_0) );

//	reckon->y = reckon->y_0 - ( r ) * ( cos( (v_right - v_left)*(reckon->t)/b + reckon->th_0 ) - cos(reckon->th_0) );
//	reckon->x = reckon->x_0 + ( r ) * ( sin( (v_right - v_left)*(reckon->t)/b + reckon->th_0 ) - sin(reckon->th_0) );


	gettimeofday(&(reckon->t1), NULL);
}

float getTargetHeading(ddrn * reckon){
	return(atan( (reckon->y_t - reckon->y) / (reckon->x_t - reckon->x) ));
}

/*
float findSpeedRatio_90(ddrn * reckon){
	//get l in terms of h (law od sines, assuiming right triangle)
	//use distance form to get h
	//get l, set radius/vel eqn equal to l

	//float h = hypotf((xt-x0), (yt-y0));

	float v_left_over_v_right = fabs( ( b - ( hypotf((reckon->x_t - reckon->x_0), (reckon->y_t - reckon->y_0)) ) * M_SQRT2 ) / ( b + ( hypotf((reckon->x_t - reckon->x_0), (reckon->y_t - reckon->y_0)) ) * M_SQRT2 ) ) ;
	return(v_left_over_v_right);
}
*/

float findSpeedRatio_Rot(ddrn * reckon, float face_angle){
	
	//return(v_left_over_v_right);
}

double radiusofcircle(double r_t, double th_t){
	double radius = (r_t) / (2 * sin(th_t));
	return(radius);
}

double findSpeedRatio(ddrn * reckon){
	
	double len = hypotf( (reckon->x_t - reckon->x) , (reckon->y_t - reckon->y) );
	double deg = atan( (reckon->x_t - reckon->x) / (reckon->y_t - reckon->y) );

	double r = radiusofcircle( len, deg );
	double vl_vr = (2*r - b) / (2*r + b);
	
	return(vl_vr);
}

#endif //RECKON_HH
