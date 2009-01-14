/*
 ** Used primarily to rechirp images which have been dechirped
 ** Thus an image of a brick wall at an oblique angle may be remapped so
 ** that all the bricks appear the same size, and thus the ``chirping''
 ** phenomenon associated with perspective projection is eliminated,
 ** and then a different chirpiness may be applied.
 ** Chirping means that the spatial frequency changes with position in the
 ** image (one raster to a digital to analog converter, listened to by on
 ** a loudspeaker gives the bird-like or bat-like ``chirping'' sound).

 ** Rechirping may be thought of as moving all the vanishing points off to
 ** infinity (via dechirp) and then moving them back to some _different_
 ** position.  This program may be used directly to re-position the perspective,
 ** or may follow chirplet_est.c; parameters being estimated from peaks in the
 ** kurtosis of the Pan-Zoom (PZ) plane of the chirplet transform of the image.
 ** (Note, in this context the ``camera metaphor'' may be useful: for example
 ** the wavelet transform is a Translation, Zoom (TZ) plane.)

 ** Steve Mann, 1992 July 9                        (Based on a dechirp2.c)
 **
*/

#include <math.h>
#include "corners2r.h"

/* Input Arguments */
#define	CORNERS_IN	prhs[0]

/* Output Arguments */

#define	RECHIRP_PARAMETERS	plhs[0]

#define	max(A, B)	((A) > (B) ? (A) : (B))
#define	min(A, B)	((A) < (B) ? (A) : (B))
#define pi 3.14159265

void corners2r(double rechirp_parameters[],
               double corners[])
{
    double u1,v1, u2,v2, u3,v3, u4,v4;
    double a_inv,b_inv,c_inv,d_inv,e_inv,f_inv,g_inv,h_inv;

    /* get corners from the input vector */
    u1 = corners[0];
    v1 = corners[1];
    u2 = corners[2];
    v2 = corners[3];
    u3 = corners[4];
    v3 = corners[5];
    u4 = corners[6];
    v4 = corners[7];

    a_inv = -(((u3*u3)*(v1*v1)*v2 - u3*u4*(v1*v1)*v2 -
               (u3*u3)*v1*(v2*v2) + u3*u4*v1*(v2*v2) -
               u2*u3*(v1*v1)*v3 + u3*u4*(v1*v1)*v3 - u1*u3*v1*v2*v3 +
               u2*u3*v1*v2*v3 + u1*u4*v1*v2*v3 - u3*u4*v1*v2*v3 +
               u1*u3*(v2*v2)*v3 - u1*u4*(v2*v2)*v3 +
               u1*u2*v1*(v3*v3) - u1*u4*v1*(v3*v3) -
               u1*u2*v2*(v3*v3) + u1*u4*v2*(v3*v3) +
               u2*u3*(v1*v1)*v4 - (u3*u3)*(v1*v1)*v4 -
               u2*u3*v1*v2*v4 + (u3*u3)*v1*v2*v4 - u1*u2*v1*v3*v4 +
               u1*u3*v1*v3*v4 + u1*u2*v2*v3*v4 - u1*u3*v2*v3*v4)/
              (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
               u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
               2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
               (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
               u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
               2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
               2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
               2*(u1*u1)*u4*(v2*v2)*v3 +
               2*u1*(u4*u4)*(v2*v2)*v3 - u3*(u4*u4)*(v2*v2)*v3 +
               u1*(u2*u2)*v1*(v3*v3) - 2*u1*u2*u4*v1*(v3*v3) +
               u1*(u4*u4)*v1*(v3*v3) - (u1*u1)*u2*v2*(v3*v3) +
               2*(u1*u1)*u4*v2*(v3*v3) -
               2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
               2*(u2*u2)*u3*(v1*v1)*v4 -
               2*u2*(u3*u3)*(v1*v1)*v4 - (u2*u2)*u4*(v1*v1)*v4 +
               (u3*u3)*u4*(v1*v1)*v4 - 2*u1*u2*u3*v1*v2*v4 +
               2*u1*(u3*u3)*v1*v2*v4 + 2*u2*u3*u4*v1*v2*v4 -
               2*(u3*u3)*u4*v1*v2*v4 + (u1*u1)*u4*(v2*v2)*v4 -
               2*u1*u3*u4*(v2*v2)*v4 + (u3*u3)*u4*(v2*v2)*v4 -
               2*u1*(u2*u2)*v1*v3*v4 + 2*u1*u2*u3*v1*v3*v4 +
               2*(u2*u2)*u4*v1*v3*v4 - 2*u2*u3*u4*v1*v3*v4 +
               2*(u1*u1)*u2*v2*v3*v4 - 2*(u1*u1)*u3*v2*v3*v4 -
               2*u1*u2*u4*v2*v3*v4 + 2*u1*u3*u4*v2*v3*v4 -
               (u1*u1)*u4*(v3*v3)*v4 + 2*u1*u2*u4*(v3*v3)*v4 -
               (u2*u2)*u4*(v3*v3)*v4 + u1*(u2*u2)*v1*(v4*v4) -
               2*(u2*u2)*u3*v1*(v4*v4) - u1*(u3*u3)*v1*(v4*v4) +
               2*u2*(u3*u3)*v1*(v4*v4) - (u1*u1)*u2*v2*(v4*v4) +
               2*u1*u2*u3*v2*(v4*v4) - u2*(u3*u3)*v2*(v4*v4) +
               (u1*u1)*u3*v3*(v4*v4) - 2*u1*u2*u3*v3*(v4*v4) +
               (u2*u2)*u3*v3*(v4*v4))) -
            (u2*u4*(v1*v1)*v3 - u3*u4*(v1*v1)*v3 - u1*u4*v1*v2*v3 -
             u2*u4*v1*v2*v3 + 2*u3*u4*v1*v2*v3 + u1*u4*(v2*v2)*v3 -
             u3*u4*(v2*v2)*v3 + u1*u4*v1*(v3*v3) - u2*u4*v1*(v3*v3) -
             u1*u4*v2*(v3*v3) + u2*u4*v2*(v3*v3) - u2*u3*(v1*v1)*v4 +
             (u3*u3)*(v1*v1)*v4 + u1*u3*v1*v2*v4 + u2*u3*v1*v2*v4 -
             2*(u3*u3)*v1*v2*v4 - u1*u3*(v2*v2)*v4 +
             (u3*u3)*(v2*v2)*v4 - u1*u3*v1*v3*v4 + u2*u3*v1*v3*v4 +
             u1*u3*v2*v3*v4 - u2*u3*v2*v3*v4)/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4)) -
            (-(u3*u4*(v1*v1)*v2) + (u4*u4)*(v1*v1)*v2 +
             u3*u4*v1*(v2*v2) - (u4*u4)*v1*(v2*v2) + u1*u4*v1*v2*v3 -
             (u4*u4)*v1*v2*v3 - u1*u4*(v2*v2)*v3 +
             (u4*u4)*(v2*v2)*v3 + u2*u3*(v1*v1)*v4 -
             u2*u4*(v1*v1)*v4 - u2*u3*v1*v2*v4 - u1*u4*v1*v2*v4 +
             u2*u4*v1*v2*v4 + u3*u4*v1*v2*v4 + u1*u4*(v2*v2)*v4 -
             u3*u4*(v2*v2)*v4 - u1*u2*v1*v3*v4 + u2*u4*v1*v3*v4 +
             u1*u2*v2*v3*v4 - u2*u4*v2*v3*v4 + u1*u2*v1*(v4*v4) -
             u2*u3*v1*(v4*v4) - u1*u2*v2*(v4*v4) + u2*u3*v2*(v4*v4))/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4)) -
            (u3*u4*(v1*v1)*v3 - (u4*u4)*(v1*v1)*v3 - u3*u4*v1*v2*v3 +
             (u4*u4)*v1*v2*v3 - u1*u4*v1*(v3*v3) +
             (u4*u4)*v1*(v3*v3) + u1*u4*v2*(v3*v3) -
             (u4*u4)*v2*(v3*v3) - (u3*u3)*(v1*v1)*v4 +
             u3*u4*(v1*v1)*v4 + (u3*u3)*v1*v2*v4 - u3*u4*v1*v2*v4 +
             u1*u3*v1*v3*v4 + u1*u4*v1*v3*v4 - 2*u3*u4*v1*v3*v4 -
             u1*u3*v2*v3*v4 - u1*u4*v2*v3*v4 + 2*u3*u4*v2*v3*v4 -
             u1*u3*v1*(v4*v4) + (u3*u3)*v1*(v4*v4) +
             u1*u3*v2*(v4*v4) - (u3*u3)*v2*(v4*v4))/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4));

    b_inv = -((-(u1*(u3*u3)*v1*v2) + u2*(u3*u3)*v1*v2 +
               u1*u3*u4*v1*v2 - u2*u3*u4*v1*v2 + u1*u2*u3*v1*v3 -
               (u2*u2)*u3*v1*v3 - u1*u3*u4*v1*v3 + u2*u3*u4*v1*v3 +
               (u1*u1)*u3*v2*v3 - u1*u2*u3*v2*v3 - (u1*u1)*u4*v2*v3 +
               u1*u2*u4*v2*v3 - (u1*u1)*u2*(v3*v3) +
               u1*(u2*u2)*(v3*v3) + (u1*u1)*u4*(v3*v3) -
               u1*u2*u4*(v3*v3) - u1*u2*u3*v1*v4 + (u2*u2)*u3*v1*v4 +
               u1*(u3*u3)*v1*v4 - u2*(u3*u3)*v1*v4 +
               (u1*u1)*u2*v3*v4 - u1*(u2*u2)*v3*v4 -
               (u1*u1)*u3*v3*v4 + u1*u2*u3*v3*v4)/
              (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
               u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
               2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
               (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
               u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
               2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
               2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
               2*(u1*u1)*u4*(v2*v2)*v3 +
               2*u1*(u4*u4)*(v2*v2)*v3 - u3*(u4*u4)*(v2*v2)*v3 +
               u1*(u2*u2)*v1*(v3*v3) - 2*u1*u2*u4*v1*(v3*v3) +
               u1*(u4*u4)*v1*(v3*v3) - (u1*u1)*u2*v2*(v3*v3) +
               2*(u1*u1)*u4*v2*(v3*v3) -
               2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
               2*(u2*u2)*u3*(v1*v1)*v4 -
               2*u2*(u3*u3)*(v1*v1)*v4 - (u2*u2)*u4*(v1*v1)*v4 +
               (u3*u3)*u4*(v1*v1)*v4 - 2*u1*u2*u3*v1*v2*v4 +
               2*u1*(u3*u3)*v1*v2*v4 + 2*u2*u3*u4*v1*v2*v4 -
               2*(u3*u3)*u4*v1*v2*v4 + (u1*u1)*u4*(v2*v2)*v4 -
               2*u1*u3*u4*(v2*v2)*v4 + (u3*u3)*u4*(v2*v2)*v4 -
               2*u1*(u2*u2)*v1*v3*v4 + 2*u1*u2*u3*v1*v3*v4 +
               2*(u2*u2)*u4*v1*v3*v4 - 2*u2*u3*u4*v1*v3*v4 +
               2*(u1*u1)*u2*v2*v3*v4 - 2*(u1*u1)*u3*v2*v3*v4 -
               2*u1*u2*u4*v2*v3*v4 + 2*u1*u3*u4*v2*v3*v4 -
               (u1*u1)*u4*(v3*v3)*v4 + 2*u1*u2*u4*(v3*v3)*v4 -
               (u2*u2)*u4*(v3*v3)*v4 + u1*(u2*u2)*v1*(v4*v4) -
               2*(u2*u2)*u3*v1*(v4*v4) - u1*(u3*u3)*v1*(v4*v4) +
               2*u2*(u3*u3)*v1*(v4*v4) - (u1*u1)*u2*v2*(v4*v4) +
               2*u1*u2*u3*v2*(v4*v4) - u2*(u3*u3)*v2*(v4*v4) +
               (u1*u1)*u3*v3*(v4*v4) - 2*u1*u2*u3*v3*(v4*v4) +
               (u2*u2)*u3*v3*(v4*v4))) -
            (-(u1*u2*u4*v1*v3) + (u2*u2)*u4*v1*v3 + u1*u3*u4*v1*v3 -
             u2*u3*u4*v1*v3 + (u1*u1)*u4*v2*v3 - u1*u2*u4*v2*v3 -
             u1*u3*u4*v2*v3 + u2*u3*u4*v2*v3 - (u1*u1)*u4*(v3*v3) +
             2*u1*u2*u4*(v3*v3) - (u2*u2)*u4*(v3*v3) +
             u1*u2*u3*v1*v4 - (u2*u2)*u3*v1*v4 - u1*(u3*u3)*v1*v4 +
             u2*(u3*u3)*v1*v4 - (u1*u1)*u3*v2*v4 + u1*u2*u3*v2*v4 +
             u1*(u3*u3)*v2*v4 - u2*(u3*u3)*v2*v4 + (u1*u1)*u3*v3*v4 -
             2*u1*u2*u3*v3*v4 + (u2*u2)*u3*v3*v4)/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4)) -
            (u1*u3*u4*v1*v2 - u2*u3*u4*v1*v2 - u1*(u4*u4)*v1*v2 +
             u2*(u4*u4)*v1*v2 - (u1*u1)*u4*v2*v3 + u1*u2*u4*v2*v3 +
             u1*(u4*u4)*v2*v3 - u2*(u4*u4)*v2*v3 - u1*u2*u3*v1*v4 +
             (u2*u2)*u3*v1*v4 + u1*u2*u4*v1*v4 - (u2*u2)*u4*v1*v4 +
             (u1*u1)*u4*v2*v4 - u1*u2*u4*v2*v4 - u1*u3*u4*v2*v4 +
             u2*u3*u4*v2*v4 + (u1*u1)*u2*v3*v4 - u1*(u2*u2)*v3*v4 -
             u1*u2*u4*v3*v4 + (u2*u2)*u4*v3*v4 - (u1*u1)*u2*(v4*v4) +
             u1*(u2*u2)*(v4*v4) + u1*u2*u3*(v4*v4) -
             (u2*u2)*u3*(v4*v4))/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4)) -
            (-(u1*u3*u4*v1*v3) + u2*u3*u4*v1*v3 + u1*(u4*u4)*v1*v3 -
             u2*(u4*u4)*v1*v3 + (u1*u1)*u4*(v3*v3) -
             u1*u2*u4*(v3*v3) - u1*(u4*u4)*(v3*v3) +
             u2*(u4*u4)*(v3*v3) + u1*(u3*u3)*v1*v4 -
             u2*(u3*u3)*v1*v4 - u1*u3*u4*v1*v4 + u2*u3*u4*v1*v4 -
             (u1*u1)*u3*v3*v4 + u1*u2*u3*v3*v4 - (u1*u1)*u4*v3*v4 +
             u1*u2*u4*v3*v4 + 2*u1*u3*u4*v3*v4 - 2*u2*u3*u4*v3*v4 +
             (u1*u1)*u3*(v4*v4) - u1*u2*u3*(v4*v4) -
             u1*(u3*u3)*(v4*v4) + u2*(u3*u3)*(v4*v4))/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4));

    c_inv = -((-(u2*(u3*u3)*(v1*v1)*v2) + u2*u3*u4*(v1*v1)*v2 +
               u1*(u3*u3)*v1*(v2*v2) - u1*u3*u4*v1*(v2*v2) +
               (u2*u2)*u3*(v1*v1)*v3 - u2*u3*u4*(v1*v1)*v3 -
               u1*u2*u4*v1*v2*v3 + u1*u3*u4*v1*v2*v3 -
               (u1*u1)*u3*(v2*v2)*v3 + (u1*u1)*u4*(v2*v2)*v3 -
               u1*(u2*u2)*v1*(v3*v3) + u1*u2*u4*v1*(v3*v3) +
               (u1*u1)*u2*v2*(v3*v3) - (u1*u1)*u4*v2*(v3*v3) -
               (u2*u2)*u3*(v1*v1)*v4 + u2*(u3*u3)*(v1*v1)*v4 +
               u1*u2*u3*v1*v2*v4 - u1*(u3*u3)*v1*v2*v4 +
               u1*(u2*u2)*v1*v3*v4 - u1*u2*u3*v1*v3*v4 -
               (u1*u1)*u2*v2*v3*v4 + (u1*u1)*u3*v2*v3*v4)/
              (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
               u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
               2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
               (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
               u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
               2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
               2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
               2*(u1*u1)*u4*(v2*v2)*v3 +
               2*u1*(u4*u4)*(v2*v2)*v3 - u3*(u4*u4)*(v2*v2)*v3 +
               u1*(u2*u2)*v1*(v3*v3) - 2*u1*u2*u4*v1*(v3*v3) +
               u1*(u4*u4)*v1*(v3*v3) - (u1*u1)*u2*v2*(v3*v3) +
               2*(u1*u1)*u4*v2*(v3*v3) -
               2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
               2*(u2*u2)*u3*(v1*v1)*v4 -
               2*u2*(u3*u3)*(v1*v1)*v4 - (u2*u2)*u4*(v1*v1)*v4 +
               (u3*u3)*u4*(v1*v1)*v4 - 2*u1*u2*u3*v1*v2*v4 +
               2*u1*(u3*u3)*v1*v2*v4 + 2*u2*u3*u4*v1*v2*v4 -
               2*(u3*u3)*u4*v1*v2*v4 + (u1*u1)*u4*(v2*v2)*v4 -
               2*u1*u3*u4*(v2*v2)*v4 + (u3*u3)*u4*(v2*v2)*v4 -
               2*u1*(u2*u2)*v1*v3*v4 + 2*u1*u2*u3*v1*v3*v4 +
               2*(u2*u2)*u4*v1*v3*v4 - 2*u2*u3*u4*v1*v3*v4 +
               2*(u1*u1)*u2*v2*v3*v4 - 2*(u1*u1)*u3*v2*v3*v4 -
               2*u1*u2*u4*v2*v3*v4 + 2*u1*u3*u4*v2*v3*v4 -
               (u1*u1)*u4*(v3*v3)*v4 + 2*u1*u2*u4*(v3*v3)*v4 -
               (u2*u2)*u4*(v3*v3)*v4 + u1*(u2*u2)*v1*(v4*v4) -
               2*(u2*u2)*u3*v1*(v4*v4) - u1*(u3*u3)*v1*(v4*v4) +
               2*u2*(u3*u3)*v1*(v4*v4) - (u1*u1)*u2*v2*(v4*v4) +
               2*u1*u2*u3*v2*(v4*v4) - u2*(u3*u3)*v2*(v4*v4) +
               (u1*u1)*u3*v3*(v4*v4) - 2*u1*u2*u3*v3*(v4*v4) +
               (u2*u2)*u3*v3*(v4*v4))) -
            (-((u2*u2)*u4*(v1*v1)*v3) + u2*u3*u4*(v1*v1)*v3 +
             2*u1*u2*u4*v1*v2*v3 - u1*u3*u4*v1*v2*v3 - u2*u3*u4*v1*v2*v3 -
             (u1*u1)*u4*(v2*v2)*v3 + u1*u3*u4*(v2*v2)*v3 -
             u1*u2*u4*v1*(v3*v3) + (u2*u2)*u4*v1*(v3*v3) +
             (u1*u1)*u4*v2*(v3*v3) - u1*u2*u4*v2*(v3*v3) +
             (u2*u2)*u3*(v1*v1)*v4 - u2*(u3*u3)*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + u1*(u3*u3)*v1*v2*v4 +
             u2*(u3*u3)*v1*v2*v4 + (u1*u1)*u3*(v2*v2)*v4 -
             u1*(u3*u3)*(v2*v2)*v4 + u1*u2*u3*v1*v3*v4 -
             (u2*u2)*u3*v1*v3*v4 - (u1*u1)*u3*v2*v3*v4 + u1*u2*u3*v2*v3*v4
            )/(u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
               u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
               2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
               (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
               u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
               2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
               2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
               2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
               u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
               2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
               (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
               2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
               2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
               (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
               2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
               2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
               (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
               (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
               2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
               2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
               2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
               2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
               2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
               u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
               u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
               (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
               u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
               2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4)) -
            (u2*u3*u4*(v1*v1)*v2 - u2*(u4*u4)*(v1*v1)*v2 -
             u1*u3*u4*v1*(v2*v2) + u1*(u4*u4)*v1*(v2*v2) -
             u1*u2*u4*v1*v2*v3 + u2*(u4*u4)*v1*v2*v3 +
             (u1*u1)*u4*(v2*v2)*v3 - u1*(u4*u4)*(v2*v2)*v3 -
             (u2*u2)*u3*(v1*v1)*v4 + (u2*u2)*u4*(v1*v1)*v4 +
             u1*u2*u3*v1*v2*v4 - u2*u3*u4*v1*v2*v4 -
             (u1*u1)*u4*(v2*v2)*v4 + u1*u3*u4*(v2*v2)*v4 +
             u1*(u2*u2)*v1*v3*v4 - (u2*u2)*u4*v1*v3*v4 -
             (u1*u1)*u2*v2*v3*v4 + u1*u2*u4*v2*v3*v4 -
             u1*(u2*u2)*v1*(v4*v4) + (u2*u2)*u3*v1*(v4*v4) +
             (u1*u1)*u2*v2*(v4*v4) - u1*u2*u3*v2*(v4*v4))/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4)) -
            (-(u2*u3*u4*(v1*v1)*v3) + u2*(u4*u4)*(v1*v1)*v3 +
             u1*u3*u4*v1*v2*v3 - u1*(u4*u4)*v1*v2*v3 +
             u1*u2*u4*v1*(v3*v3) - u2*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u4*v2*(v3*v3) + u1*(u4*u4)*v2*(v3*v3) +
             u2*(u3*u3)*(v1*v1)*v4 - u2*u3*u4*(v1*v1)*v4 -
             u1*(u3*u3)*v1*v2*v4 + u1*u3*u4*v1*v2*v4 - u1*u2*u3*v1*v3*v4 -
             u1*u2*u4*v1*v3*v4 + 2*u2*u3*u4*v1*v3*v4 + (u1*u1)*u3*v2*v3*v4 +
             (u1*u1)*u4*v2*v3*v4 - 2*u1*u3*u4*v2*v3*v4 +
             u1*u2*u3*v1*(v4*v4) - u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u3*v2*(v4*v4) + u1*(u3*u3)*v2*(v4*v4))/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4));

    d_inv = -((u2*u3*(v1*v1)*v2 - u2*u4*(v1*v1)*v2 -
               u1*u3*v1*(v2*v2) + u1*u4*v1*(v2*v2) -
               (u2*u2)*(v1*v1)*v3 + u2*u4*(v1*v1)*v3 +
               u1*u2*v1*v2*v3 - u2*u3*v1*v2*v3 - u1*u4*v1*v2*v3 +
               u2*u4*v1*v2*v3 + u1*u3*(v2*v2)*v3 - u1*u4*(v2*v2)*v3 +
               (u2*u2)*v1*(v3*v3) - u2*u4*v1*(v3*v3) -
               u1*u2*v2*(v3*v3) + u1*u4*v2*(v3*v3) +
               (u2*u2)*(v1*v1)*v4 - u2*u3*(v1*v1)*v4 -
               u1*u2*v1*v2*v4 + u1*u3*v1*v2*v4 - (u2*u2)*v1*v3*v4 +
               u2*u3*v1*v3*v4 + u1*u2*v2*v3*v4 - u1*u3*v2*v3*v4)/
              (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
               u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
               2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
               (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
               u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
               2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
               2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
               2*(u1*u1)*u4*(v2*v2)*v3 +
               2*u1*(u4*u4)*(v2*v2)*v3 - u3*(u4*u4)*(v2*v2)*v3 +
               u1*(u2*u2)*v1*(v3*v3) - 2*u1*u2*u4*v1*(v3*v3) +
               u1*(u4*u4)*v1*(v3*v3) - (u1*u1)*u2*v2*(v3*v3) +
               2*(u1*u1)*u4*v2*(v3*v3) -
               2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
               2*(u2*u2)*u3*(v1*v1)*v4 -
               2*u2*(u3*u3)*(v1*v1)*v4 - (u2*u2)*u4*(v1*v1)*v4 +
               (u3*u3)*u4*(v1*v1)*v4 - 2*u1*u2*u3*v1*v2*v4 +
               2*u1*(u3*u3)*v1*v2*v4 + 2*u2*u3*u4*v1*v2*v4 -
               2*(u3*u3)*u4*v1*v2*v4 + (u1*u1)*u4*(v2*v2)*v4 -
               2*u1*u3*u4*(v2*v2)*v4 + (u3*u3)*u4*(v2*v2)*v4 -
               2*u1*(u2*u2)*v1*v3*v4 + 2*u1*u2*u3*v1*v3*v4 +
               2*(u2*u2)*u4*v1*v3*v4 - 2*u2*u3*u4*v1*v3*v4 +
               2*(u1*u1)*u2*v2*v3*v4 - 2*(u1*u1)*u3*v2*v3*v4 -
               2*u1*u2*u4*v2*v3*v4 + 2*u1*u3*u4*v2*v3*v4 -
               (u1*u1)*u4*(v3*v3)*v4 + 2*u1*u2*u4*(v3*v3)*v4 -
               (u2*u2)*u4*(v3*v3)*v4 + u1*(u2*u2)*v1*(v4*v4) -
               2*(u2*u2)*u3*v1*(v4*v4) - u1*(u3*u3)*v1*(v4*v4) +
               2*u2*(u3*u3)*v1*(v4*v4) - (u1*u1)*u2*v2*(v4*v4) +
               2*u1*u2*u3*v2*(v4*v4) - u2*(u3*u3)*v2*(v4*v4) +
               (u1*u1)*u3*v3*(v4*v4) - 2*u1*u2*u3*v3*(v4*v4) +
               (u2*u2)*u3*v3*(v4*v4))) -
            (u2*u4*(v1*v1)*v2 - u3*u4*(v1*v1)*v2 - u1*u4*v1*(v2*v2) +
             u3*u4*v1*(v2*v2) + u1*u4*v1*v2*v3 - 2*u2*u4*v1*v2*v3 +
             u3*u4*v1*v2*v3 + u1*u4*(v2*v2)*v3 - u3*u4*(v2*v2)*v3 -
             u1*u4*v2*(v3*v3) + u2*u4*v2*(v3*v3) -
             (u2*u2)*(v1*v1)*v4 + u2*u3*(v1*v1)*v4 + u1*u2*v1*v2*v4 -
             u2*u3*v1*v2*v4 - u1*u2*v1*v3*v4 + 2*(u2*u2)*v1*v3*v4 -
             u2*u3*v1*v3*v4 - u1*u2*v2*v3*v4 + u2*u3*v2*v3*v4 +
             u1*u2*(v3*v3)*v4 - (u2*u2)*(v3*v3)*v4)/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4)) -
            (-(u2*u4*(v1*v1)*v2) + (u4*u4)*(v1*v1)*v2 +
             u1*u4*v1*(v2*v2) - (u4*u4)*v1*(v2*v2) + u2*u4*v1*v2*v3 -
             (u4*u4)*v1*v2*v3 - u1*u4*(v2*v2)*v3 +
             (u4*u4)*(v2*v2)*v3 + (u2*u2)*(v1*v1)*v4 -
             u2*u4*(v1*v1)*v4 - u1*u2*v1*v2*v4 - u1*u4*v1*v2*v4 +
             2*u2*u4*v1*v2*v4 - (u2*u2)*v1*v3*v4 + u2*u4*v1*v3*v4 +
             u1*u2*v2*v3*v4 + u1*u4*v2*v3*v4 - 2*u2*u4*v2*v3*v4 +
             u1*u2*v1*(v4*v4) - (u2*u2)*v1*(v4*v4) -
             u1*u2*v3*(v4*v4) + (u2*u2)*v3*(v4*v4))/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4)) -
            (u2*u4*(v1*v1)*v3 - (u4*u4)*(v1*v1)*v3 - u1*u4*v1*v2*v3 +
             (u4*u4)*v1*v2*v3 - u2*u4*v1*(v3*v3) +
             (u4*u4)*v1*(v3*v3) + u1*u4*v2*(v3*v3) -
             (u4*u4)*v2*(v3*v3) - u2*u3*(v1*v1)*v4 +
             u3*u4*(v1*v1)*v4 + u1*u3*v1*v2*v4 - u3*u4*v1*v2*v4 +
             u2*u3*v1*v3*v4 + u1*u4*v1*v3*v4 - u2*u4*v1*v3*v4 - u3*u4*v1*v3*v4 -
             u1*u3*v2*v3*v4 + u3*u4*v2*v3*v4 - u1*u4*(v3*v3)*v4 +
             u2*u4*(v3*v3)*v4 - u1*u3*v1*(v4*v4) + u2*u3*v1*(v4*v4) +
             u1*u3*v3*(v4*v4) - u2*u3*v3*(v4*v4))/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4));

    e_inv = -((-(u1*u2*u3*v1*v2) + u2*(u3*u3)*v1*v2 + u1*u2*u4*v1*v2 -
               u2*u3*u4*v1*v2 + (u1*u1)*u3*(v2*v2) -
               u1*(u3*u3)*(v2*v2) - (u1*u1)*u4*(v2*v2) +
               u1*u3*u4*(v2*v2) + u1*(u2*u2)*v1*v3 -
               (u2*u2)*u3*v1*v3 - u1*u2*u4*v1*v3 + u2*u3*u4*v1*v3 -
               (u1*u1)*u2*v2*v3 + u1*u2*u3*v2*v3 + (u1*u1)*u4*v2*v3 -
               u1*u3*u4*v2*v3 - u1*(u2*u2)*v1*v4 + u1*u2*u3*v1*v4 +
               (u2*u2)*u3*v1*v4 - u2*(u3*u3)*v1*v4 +
               (u1*u1)*u2*v2*v4 - (u1*u1)*u3*v2*v4 - u1*u2*u3*v2*v4 +
               u1*(u3*u3)*v2*v4)/
              (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
               u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
               2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
               (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
               u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
               2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
               2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
               2*(u1*u1)*u4*(v2*v2)*v3 +
               2*u1*(u4*u4)*(v2*v2)*v3 - u3*(u4*u4)*(v2*v2)*v3 +
               u1*(u2*u2)*v1*(v3*v3) - 2*u1*u2*u4*v1*(v3*v3) +
               u1*(u4*u4)*v1*(v3*v3) - (u1*u1)*u2*v2*(v3*v3) +
               2*(u1*u1)*u4*v2*(v3*v3) -
               2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
               2*(u2*u2)*u3*(v1*v1)*v4 -
               2*u2*(u3*u3)*(v1*v1)*v4 - (u2*u2)*u4*(v1*v1)*v4 +
               (u3*u3)*u4*(v1*v1)*v4 - 2*u1*u2*u3*v1*v2*v4 +
               2*u1*(u3*u3)*v1*v2*v4 + 2*u2*u3*u4*v1*v2*v4 -
               2*(u3*u3)*u4*v1*v2*v4 + (u1*u1)*u4*(v2*v2)*v4 -
               2*u1*u3*u4*(v2*v2)*v4 + (u3*u3)*u4*(v2*v2)*v4 -
               2*u1*(u2*u2)*v1*v3*v4 + 2*u1*u2*u3*v1*v3*v4 +
               2*(u2*u2)*u4*v1*v3*v4 - 2*u2*u3*u4*v1*v3*v4 +
               2*(u1*u1)*u2*v2*v3*v4 - 2*(u1*u1)*u3*v2*v3*v4 -
               2*u1*u2*u4*v2*v3*v4 + 2*u1*u3*u4*v2*v3*v4 -
               (u1*u1)*u4*(v3*v3)*v4 + 2*u1*u2*u4*(v3*v3)*v4 -
               (u2*u2)*u4*(v3*v3)*v4 + u1*(u2*u2)*v1*(v4*v4) -
               2*(u2*u2)*u3*v1*(v4*v4) - u1*(u3*u3)*v1*(v4*v4) +
               2*u2*(u3*u3)*v1*(v4*v4) - (u1*u1)*u2*v2*(v4*v4) +
               2*u1*u2*u3*v2*(v4*v4) - u2*(u3*u3)*v2*(v4*v4) +
               (u1*u1)*u3*v3*(v4*v4) - 2*u1*u2*u3*v3*(v4*v4) +
               (u2*u2)*u3*v3*(v4*v4))) -
            (-(u1*u2*u4*v1*v2) + u1*u3*u4*v1*v2 + u2*u3*u4*v1*v2 -
             (u3*u3)*u4*v1*v2 + (u1*u1)*u4*(v2*v2) -
             2*u1*u3*u4*(v2*v2) + (u3*u3)*u4*(v2*v2) -
             (u1*u1)*u4*v2*v3 + u1*u2*u4*v2*v3 + u1*u3*u4*v2*v3 -
             u2*u3*u4*v2*v3 + u1*(u2*u2)*v1*v4 - u1*u2*u3*v1*v4 -
             (u2*u2)*u3*v1*v4 + u2*(u3*u3)*v1*v4 - (u1*u1)*u2*v2*v4 +
             2*u1*u2*u3*v2*v4 - u2*(u3*u3)*v2*v4 + (u1*u1)*u2*v3*v4 -
             u1*(u2*u2)*v3*v4 - u1*u2*u3*v3*v4 + (u2*u2)*u3*v3*v4)/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4)) -
            (u1*u2*u4*v1*v2 - u2*u3*u4*v1*v2 - u1*(u4*u4)*v1*v2 +
             u3*(u4*u4)*v1*v2 - (u1*u1)*u4*(v2*v2) +
             u1*u3*u4*(v2*v2) + u1*(u4*u4)*(v2*v2) -
             u3*(u4*u4)*(v2*v2) - u1*(u2*u2)*v1*v4 +
             (u2*u2)*u3*v1*v4 + u1*u2*u4*v1*v4 - u2*u3*u4*v1*v4 +
             (u1*u1)*u2*v2*v4 - u1*u2*u3*v2*v4 + (u1*u1)*u4*v2*v4 -
             2*u1*u2*u4*v2*v4 - u1*u3*u4*v2*v4 + 2*u2*u3*u4*v2*v4 -
             (u1*u1)*u2*(v4*v4) + u1*(u2*u2)*(v4*v4) +
             u1*u2*u3*(v4*v4) - (u2*u2)*u3*(v4*v4))/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4)) -
            (-(u1*u2*u4*v1*v3) + u2*u3*u4*v1*v3 + u1*(u4*u4)*v1*v3 -
             u3*(u4*u4)*v1*v3 + (u1*u1)*u4*v2*v3 - u1*u3*u4*v2*v3 -
             u1*(u4*u4)*v2*v3 + u3*(u4*u4)*v2*v3 + u1*u2*u3*v1*v4 -
             u2*(u3*u3)*v1*v4 - u1*u3*u4*v1*v4 + (u3*u3)*u4*v1*v4 -
             (u1*u1)*u3*v2*v4 + u1*(u3*u3)*v2*v4 + u1*u3*u4*v2*v4 -
             (u3*u3)*u4*v2*v4 - (u1*u1)*u4*v3*v4 + u1*u2*u4*v3*v4 +
             u1*u3*u4*v3*v4 - u2*u3*u4*v3*v4 + (u1*u1)*u3*(v4*v4) -
             u1*u2*u3*(v4*v4) - u1*(u3*u3)*(v4*v4) +
             u2*(u3*u3)*(v4*v4))/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4));

    f_inv = -((-(u2*(u3*u3)*(v1*v1)*v2) + u2*u3*u4*(v1*v1)*v2 +
               u1*(u3*u3)*v1*(v2*v2) - u1*u3*u4*v1*(v2*v2) +
               (u2*u2)*u3*(v1*v1)*v3 - u2*u3*u4*(v1*v1)*v3 -
               u1*u2*u4*v1*v2*v3 + u1*u3*u4*v1*v2*v3 -
               (u1*u1)*u3*(v2*v2)*v3 + (u1*u1)*u4*(v2*v2)*v3 -
               u1*(u2*u2)*v1*(v3*v3) + u1*u2*u4*v1*(v3*v3) +
               (u1*u1)*u2*v2*(v3*v3) - (u1*u1)*u4*v2*(v3*v3) -
               (u2*u2)*u3*(v1*v1)*v4 + u2*(u3*u3)*(v1*v1)*v4 +
               u1*u2*u3*v1*v2*v4 - u1*(u3*u3)*v1*v2*v4 +
               u1*(u2*u2)*v1*v3*v4 - u1*u2*u3*v1*v3*v4 -
               (u1*u1)*u2*v2*v3*v4 + (u1*u1)*u3*v2*v3*v4)/
              (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
               u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
               2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
               (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
               u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
               2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
               2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
               2*(u1*u1)*u4*(v2*v2)*v3 +
               2*u1*(u4*u4)*(v2*v2)*v3 - u3*(u4*u4)*(v2*v2)*v3 +
               u1*(u2*u2)*v1*(v3*v3) - 2*u1*u2*u4*v1*(v3*v3) +
               u1*(u4*u4)*v1*(v3*v3) - (u1*u1)*u2*v2*(v3*v3) +
               2*(u1*u1)*u4*v2*(v3*v3) -
               2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
               2*(u2*u2)*u3*(v1*v1)*v4 -
               2*u2*(u3*u3)*(v1*v1)*v4 - (u2*u2)*u4*(v1*v1)*v4 +
               (u3*u3)*u4*(v1*v1)*v4 - 2*u1*u2*u3*v1*v2*v4 +
               2*u1*(u3*u3)*v1*v2*v4 + 2*u2*u3*u4*v1*v2*v4 -
               2*(u3*u3)*u4*v1*v2*v4 + (u1*u1)*u4*(v2*v2)*v4 -
               2*u1*u3*u4*(v2*v2)*v4 + (u3*u3)*u4*(v2*v2)*v4 -
               2*u1*(u2*u2)*v1*v3*v4 + 2*u1*u2*u3*v1*v3*v4 +
               2*(u2*u2)*u4*v1*v3*v4 - 2*u2*u3*u4*v1*v3*v4 +
               2*(u1*u1)*u2*v2*v3*v4 - 2*(u1*u1)*u3*v2*v3*v4 -
               2*u1*u2*u4*v2*v3*v4 + 2*u1*u3*u4*v2*v3*v4 -
               (u1*u1)*u4*(v3*v3)*v4 + 2*u1*u2*u4*(v3*v3)*v4 -
               (u2*u2)*u4*(v3*v3)*v4 + u1*(u2*u2)*v1*(v4*v4) -
               2*(u2*u2)*u3*v1*(v4*v4) - u1*(u3*u3)*v1*(v4*v4) +
               2*u2*(u3*u3)*v1*(v4*v4) - (u1*u1)*u2*v2*(v4*v4) +
               2*u1*u2*u3*v2*(v4*v4) - u2*(u3*u3)*v2*(v4*v4) +
               (u1*u1)*u3*v3*(v4*v4) - 2*u1*u2*u3*v3*(v4*v4) +
               (u2*u2)*u3*v3*(v4*v4))) -
            (-(u2*u3*u4*(v1*v1)*v2) + (u3*u3)*u4*(v1*v1)*v2 +
             u1*u3*u4*v1*(v2*v2) - (u3*u3)*u4*v1*(v2*v2) +
             u1*u2*u4*v1*v2*v3 - 2*u1*u3*u4*v1*v2*v3 + u2*u3*u4*v1*v2*v3 -
             (u1*u1)*u4*(v2*v2)*v3 + u1*u3*u4*(v2*v2)*v3 +
             (u1*u1)*u4*v2*(v3*v3) - u1*u2*u4*v2*(v3*v3) +
             (u2*u2)*u3*(v1*v1)*v4 - u2*(u3*u3)*(v1*v1)*v4 -
             u1*u2*u3*v1*v2*v4 + u2*(u3*u3)*v1*v2*v4 -
             u1*(u2*u2)*v1*v3*v4 + 2*u1*u2*u3*v1*v3*v4 -
             (u2*u2)*u3*v1*v3*v4 + (u1*u1)*u2*v2*v3*v4 -
             u1*u2*u3*v2*v3*v4 - (u1*u1)*u2*(v3*v3)*v4 +
             u1*(u2*u2)*(v3*v3)*v4)/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4)) -
            (u2*u3*u4*(v1*v1)*v2 - u3*(u4*u4)*(v1*v1)*v2 -
             u1*u3*u4*v1*(v2*v2) + u3*(u4*u4)*v1*(v2*v2) -
             u1*u2*u4*v1*v2*v3 + u1*(u4*u4)*v1*v2*v3 +
             (u1*u1)*u4*(v2*v2)*v3 - u1*(u4*u4)*(v2*v2)*v3 -
             (u2*u2)*u3*(v1*v1)*v4 + u2*u3*u4*(v1*v1)*v4 +
             u1*u2*u3*v1*v2*v4 + u1*u3*u4*v1*v2*v4 - 2*u2*u3*u4*v1*v2*v4 +
             u1*(u2*u2)*v1*v3*v4 - u1*u2*u4*v1*v3*v4 -
             (u1*u1)*u2*v2*v3*v4 - (u1*u1)*u4*v2*v3*v4 +
             2*u1*u2*u4*v2*v3*v4 - u1*u2*u3*v1*(v4*v4) +
             (u2*u2)*u3*v1*(v4*v4) + (u1*u1)*u2*v3*(v4*v4) -
             u1*(u2*u2)*v3*(v4*v4))/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4)) -
            (-(u2*u3*u4*(v1*v1)*v3) + u3*(u4*u4)*(v1*v1)*v3 +
             u1*u3*u4*v1*v2*v3 - u3*(u4*u4)*v1*v2*v3 +
             u1*u2*u4*v1*(v3*v3) - u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u4*v2*(v3*v3) + u1*(u4*u4)*v2*(v3*v3) +
             u2*(u3*u3)*(v1*v1)*v4 - (u3*u3)*u4*(v1*v1)*v4 -
             u1*(u3*u3)*v1*v2*v4 + (u3*u3)*u4*v1*v2*v4 -
             u1*u2*u3*v1*v3*v4 + u2*u3*u4*v1*v3*v4 + (u1*u1)*u3*v2*v3*v4 -
             u1*u3*u4*v2*v3*v4 + (u1*u1)*u4*(v3*v3)*v4 -
             u1*u2*u4*(v3*v3)*v4 + u1*(u3*u3)*v1*(v4*v4) -
             u2*(u3*u3)*v1*(v4*v4) - (u1*u1)*u3*v3*(v4*v4) +
             u1*u2*u3*v3*(v4*v4))/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4));

    g_inv = -((-((u2*u2)*(v1*v1)*v3) + u2*u3*(v1*v1)*v3 +
               u2*u4*(v1*v1)*v3 - u3*u4*(v1*v1)*v3 + 2*u1*u2*v1*v2*v3 -
               u1*u3*v1*v2*v3 - u2*u3*v1*v2*v3 - u1*u4*v1*v2*v3 -
               u2*u4*v1*v2*v3 + 2*u3*u4*v1*v2*v3 - (u1*u1)*(v2*v2)*v3 +
               u1*u3*(v2*v2)*v3 + u1*u4*(v2*v2)*v3 -
               u3*u4*(v2*v2)*v3 - u1*u2*v1*(v3*v3) +
               (u2*u2)*v1*(v3*v3) + u1*u4*v1*(v3*v3) -
               u2*u4*v1*(v3*v3) + (u1*u1)*v2*(v3*v3) -
               u1*u2*v2*(v3*v3) - u1*u4*v2*(v3*v3) +
               u2*u4*v2*(v3*v3) + (u2*u2)*(v1*v1)*v4 -
               2*u2*u3*(v1*v1)*v4 + (u3*u3)*(v1*v1)*v4 -
               2*u1*u2*v1*v2*v4 + 2*u1*u3*v1*v2*v4 + 2*u2*u3*v1*v2*v4 -
               2*(u3*u3)*v1*v2*v4 + (u1*u1)*(v2*v2)*v4 -
               2*u1*u3*(v2*v2)*v4 + (u3*u3)*(v2*v2)*v4 +
               u1*u2*v1*v3*v4 - (u2*u2)*v1*v3*v4 - u1*u3*v1*v3*v4 +
               u2*u3*v1*v3*v4 - (u1*u1)*v2*v3*v4 + u1*u2*v2*v3*v4 +
               u1*u3*v2*v3*v4 - u2*u3*v2*v3*v4)/
              (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
               u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
               2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
               (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
               u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
               2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
               2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
               2*(u1*u1)*u4*(v2*v2)*v3 +
               2*u1*(u4*u4)*(v2*v2)*v3 - u3*(u4*u4)*(v2*v2)*v3 +
               u1*(u2*u2)*v1*(v3*v3) - 2*u1*u2*u4*v1*(v3*v3) +
               u1*(u4*u4)*v1*(v3*v3) - (u1*u1)*u2*v2*(v3*v3) +
               2*(u1*u1)*u4*v2*(v3*v3) -
               2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
               2*(u2*u2)*u3*(v1*v1)*v4 -
               2*u2*(u3*u3)*(v1*v1)*v4 - (u2*u2)*u4*(v1*v1)*v4 +
               (u3*u3)*u4*(v1*v1)*v4 - 2*u1*u2*u3*v1*v2*v4 +
               2*u1*(u3*u3)*v1*v2*v4 + 2*u2*u3*u4*v1*v2*v4 -
               2*(u3*u3)*u4*v1*v2*v4 + (u1*u1)*u4*(v2*v2)*v4 -
               2*u1*u3*u4*(v2*v2)*v4 + (u3*u3)*u4*(v2*v2)*v4 -
               2*u1*(u2*u2)*v1*v3*v4 + 2*u1*u2*u3*v1*v3*v4 +
               2*(u2*u2)*u4*v1*v3*v4 - 2*u2*u3*u4*v1*v3*v4 +
               2*(u1*u1)*u2*v2*v3*v4 - 2*(u1*u1)*u3*v2*v3*v4 -
               2*u1*u2*u4*v2*v3*v4 + 2*u1*u3*u4*v2*v3*v4 -
               (u1*u1)*u4*(v3*v3)*v4 + 2*u1*u2*u4*(v3*v3)*v4 -
               (u2*u2)*u4*(v3*v3)*v4 + u1*(u2*u2)*v1*(v4*v4) -
               2*(u2*u2)*u3*v1*(v4*v4) - u1*(u3*u3)*v1*(v4*v4) +
               2*u2*(u3*u3)*v1*(v4*v4) - (u1*u1)*u2*v2*(v4*v4) +
               2*u1*u2*u3*v2*(v4*v4) - u2*(u3*u3)*v2*(v4*v4) +
               (u1*u1)*u3*v3*(v4*v4) - 2*u1*u2*u3*v3*(v4*v4) +
               (u2*u2)*u3*v3*(v4*v4))) -
            (-(u2*u3*(v1*v1)*v2) + (u3*u3)*(v1*v1)*v2 +
             u2*u4*(v1*v1)*v2 - u3*u4*(v1*v1)*v2 + u1*u3*v1*(v2*v2) -
             (u3*u3)*v1*(v2*v2) - u1*u4*v1*(v2*v2) +
             u3*u4*v1*(v2*v2) + u1*u2*v1*v2*v3 - 2*u1*u3*v1*v2*v3 +
             u2*u3*v1*v2*v3 + u1*u4*v1*v2*v3 - 2*u2*u4*v1*v2*v3 +
             u3*u4*v1*v2*v3 - (u1*u1)*(v2*v2)*v3 + u1*u3*(v2*v2)*v3 +
             u1*u4*(v2*v2)*v3 - u3*u4*(v2*v2)*v3 +
             (u1*u1)*v2*(v3*v3) - u1*u2*v2*(v3*v3) -
             u1*u4*v2*(v3*v3) + u2*u4*v2*(v3*v3) -
             (u2*u2)*(v1*v1)*v4 + 2*u2*u3*(v1*v1)*v4 -
             (u3*u3)*(v1*v1)*v4 + u1*u2*v1*v2*v4 - u1*u3*v1*v2*v4 -
             u2*u3*v1*v2*v4 + (u3*u3)*v1*v2*v4 - 2*u1*u2*v1*v3*v4 +
             2*(u2*u2)*v1*v3*v4 + 2*u1*u3*v1*v3*v4 - 2*u2*u3*v1*v3*v4 +
             (u1*u1)*v2*v3*v4 - u1*u2*v2*v3*v4 - u1*u3*v2*v3*v4 +
             u2*u3*v2*v3*v4 - (u1*u1)*(v3*v3)*v4 +
             2*u1*u2*(v3*v3)*v4 - (u2*u2)*(v3*v3)*v4)/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4)) -
            (-(u2*u3*(v1*v1)*v3) + u2*u4*(v1*v1)*v3 +
             u3*u4*(v1*v1)*v3 - (u4*u4)*(v1*v1)*v3 + u1*u3*v1*v2*v3 -
             u1*u4*v1*v2*v3 - u3*u4*v1*v2*v3 + (u4*u4)*v1*v2*v3 +
             u1*u2*v1*(v3*v3) - u1*u4*v1*(v3*v3) - u2*u4*v1*(v3*v3) +
             (u4*u4)*v1*(v3*v3) - (u1*u1)*v2*(v3*v3) +
             2*u1*u4*v2*(v3*v3) - (u4*u4)*v2*(v3*v3) +
             u2*u3*(v1*v1)*v4 - (u3*u3)*(v1*v1)*v4 -
             u2*u4*(v1*v1)*v4 + u3*u4*(v1*v1)*v4 - u1*u3*v1*v2*v4 +
             (u3*u3)*v1*v2*v4 + u1*u4*v1*v2*v4 - u3*u4*v1*v2*v4 -
             2*u1*u2*v1*v3*v4 + u1*u3*v1*v3*v4 + u2*u3*v1*v3*v4 +
             u1*u4*v1*v3*v4 + u2*u4*v1*v3*v4 - 2*u3*u4*v1*v3*v4 +
             2*(u1*u1)*v2*v3*v4 - 2*u1*u3*v2*v3*v4 - 2*u1*u4*v2*v3*v4 +
             2*u3*u4*v2*v3*v4 + u1*u2*v1*(v4*v4) - u1*u3*v1*(v4*v4) -
             u2*u3*v1*(v4*v4) + (u3*u3)*v1*(v4*v4) -
             (u1*u1)*v2*(v4*v4) + 2*u1*u3*v2*(v4*v4) -
             (u3*u3)*v2*(v4*v4))/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4)) -
            (u2*u3*(v1*v1)*v2 - u2*u4*(v1*v1)*v2 - u3*u4*(v1*v1)*v2 +
             (u4*u4)*(v1*v1)*v2 - u1*u3*v1*(v2*v2) +
             u1*u4*v1*(v2*v2) + u3*u4*v1*(v2*v2) -
             (u4*u4)*v1*(v2*v2) - u1*u2*v1*v2*v3 + u1*u4*v1*v2*v3 +
             u2*u4*v1*v2*v3 - (u4*u4)*v1*v2*v3 + (u1*u1)*(v2*v2)*v3 -
             2*u1*u4*(v2*v2)*v3 + (u4*u4)*(v2*v2)*v3 +
             (u2*u2)*(v1*v1)*v4 - u2*u3*(v1*v1)*v4 -
             u2*u4*(v1*v1)*v4 + u3*u4*(v1*v1)*v4 - u1*u2*v1*v2*v4 +
             2*u1*u3*v1*v2*v4 - u2*u3*v1*v2*v4 - u1*u4*v1*v2*v4 +
             2*u2*u4*v1*v2*v4 - u3*u4*v1*v2*v4 + u1*u2*v1*v3*v4 -
             (u2*u2)*v1*v3*v4 - u1*u4*v1*v3*v4 + u2*u4*v1*v3*v4 -
             2*(u1*u1)*v2*v3*v4 + 2*u1*u2*v2*v3*v4 + 2*u1*u4*v2*v3*v4 -
             2*u2*u4*v2*v3*v4 + u1*u2*v1*(v4*v4) -
             (u2*u2)*v1*(v4*v4) - u1*u3*v1*(v4*v4) +
             u2*u3*v1*(v4*v4) + (u1*u1)*v3*(v4*v4) -
             2*u1*u2*v3*(v4*v4) + (u2*u2)*v3*(v4*v4))/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4));

    h_inv = -(((u2*u2)*u3*(v1*v1) - u2*(u3*u3)*(v1*v1) -
               (u2*u2)*u4*(v1*v1) + u2*u3*u4*(v1*v1) -
               2*u1*u2*u3*v1*v2 + u1*(u3*u3)*v1*v2 + u2*(u3*u3)*v1*v2 +
               2*u1*u2*u4*v1*v2 - u1*u3*u4*v1*v2 - u2*u3*u4*v1*v2 +
               (u1*u1)*u3*(v2*v2) - u1*(u3*u3)*(v2*v2) -
               (u1*u1)*u4*(v2*v2) + u1*u3*u4*(v2*v2) +
               u1*u2*u3*v1*v3 - (u2*u2)*u3*v1*v3 - 2*u1*u2*u4*v1*v3 +
               2*(u2*u2)*u4*v1*v3 + u1*u3*u4*v1*v3 - u2*u3*u4*v1*v3 -
               (u1*u1)*u3*v2*v3 + u1*u2*u3*v2*v3 + 2*(u1*u1)*u4*v2*v3 -
               2*u1*u2*u4*v2*v3 - u1*u3*u4*v2*v3 + u2*u3*u4*v2*v3 -
               (u1*u1)*u4*(v3*v3) + 2*u1*u2*u4*(v3*v3) -
               (u2*u2)*u4*(v3*v3) + u1*u2*u3*v1*v4 -
               (u2*u2)*u3*v1*v4 - u1*(u3*u3)*v1*v4 +
               u2*(u3*u3)*v1*v4 - (u1*u1)*u3*v2*v4 + u1*u2*u3*v2*v4 +
               u1*(u3*u3)*v2*v4 - u2*(u3*u3)*v2*v4 +
               (u1*u1)*u3*v3*v4 - 2*u1*u2*u3*v3*v4 + (u2*u2)*u3*v3*v4)/
              (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
               u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
               2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
               (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
               u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
               2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
               2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
               2*(u1*u1)*u4*(v2*v2)*v3 +
               2*u1*(u4*u4)*(v2*v2)*v3 - u3*(u4*u4)*(v2*v2)*v3 +
               u1*(u2*u2)*v1*(v3*v3) - 2*u1*u2*u4*v1*(v3*v3) +
               u1*(u4*u4)*v1*(v3*v3) - (u1*u1)*u2*v2*(v3*v3) +
               2*(u1*u1)*u4*v2*(v3*v3) -
               2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
               2*(u2*u2)*u3*(v1*v1)*v4 -
               2*u2*(u3*u3)*(v1*v1)*v4 - (u2*u2)*u4*(v1*v1)*v4 +
               (u3*u3)*u4*(v1*v1)*v4 - 2*u1*u2*u3*v1*v2*v4 +
               2*u1*(u3*u3)*v1*v2*v4 + 2*u2*u3*u4*v1*v2*v4 -
               2*(u3*u3)*u4*v1*v2*v4 + (u1*u1)*u4*(v2*v2)*v4 -
               2*u1*u3*u4*(v2*v2)*v4 + (u3*u3)*u4*(v2*v2)*v4 -
               2*u1*(u2*u2)*v1*v3*v4 + 2*u1*u2*u3*v1*v3*v4 +
               2*(u2*u2)*u4*v1*v3*v4 - 2*u2*u3*u4*v1*v3*v4 +
               2*(u1*u1)*u2*v2*v3*v4 - 2*(u1*u1)*u3*v2*v3*v4 -
               2*u1*u2*u4*v2*v3*v4 + 2*u1*u3*u4*v2*v3*v4 -
               (u1*u1)*u4*(v3*v3)*v4 + 2*u1*u2*u4*(v3*v3)*v4 -
               (u2*u2)*u4*(v3*v3)*v4 + u1*(u2*u2)*v1*(v4*v4) -
               2*(u2*u2)*u3*v1*(v4*v4) - u1*(u3*u3)*v1*(v4*v4) +
               2*u2*(u3*u3)*v1*(v4*v4) - (u1*u1)*u2*v2*(v4*v4) +
               2*u1*u2*u3*v2*(v4*v4) - u2*(u3*u3)*v2*(v4*v4) +
               (u1*u1)*u3*v3*(v4*v4) - 2*u1*u2*u3*v3*(v4*v4) +
               (u2*u2)*u3*v3*(v4*v4))) -
            ((u2*u2)*u3*(v1*v1) - u2*(u3*u3)*(v1*v1) -
             u2*u3*u4*(v1*v1) + (u3*u3)*u4*(v1*v1) - u1*u2*u3*v1*v2 +
             u2*(u3*u3)*v1*v2 - u1*u2*u4*v1*v2 + 2*u1*u3*u4*v1*v2 +
             u2*u3*u4*v1*v2 - 2*(u3*u3)*u4*v1*v2 +
             (u1*u1)*u4*(v2*v2) - 2*u1*u3*u4*(v2*v2) +
             (u3*u3)*u4*(v2*v2) - u1*(u2*u2)*v1*v3 +
             2*u1*u2*u3*v1*v3 - (u2*u2)*u3*v1*v3 + u1*u2*u4*v1*v3 -
             2*u1*u3*u4*v1*v3 + u2*u3*u4*v1*v3 + (u1*u1)*u2*v2*v3 -
             u1*u2*u3*v2*v3 - 2*(u1*u1)*u4*v2*v3 + u1*u2*u4*v2*v3 +
             2*u1*u3*u4*v2*v3 - u2*u3*u4*v2*v3 - (u1*u1)*u2*(v3*v3) +
             u1*(u2*u2)*(v3*v3) + (u1*u1)*u4*(v3*v3) -
             u1*u2*u4*(v3*v3) + u1*(u2*u2)*v1*v4 - u1*u2*u3*v1*v4 -
             (u2*u2)*u3*v1*v4 + u2*(u3*u3)*v1*v4 - (u1*u1)*u2*v2*v4 +
             2*u1*u2*u3*v2*v4 - u2*(u3*u3)*v2*v4 + (u1*u1)*u2*v3*v4 -
             u1*(u2*u2)*v3*v4 - u1*u2*u3*v3*v4 + (u2*u2)*u3*v3*v4)/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4)) -
            (-((u2*u2)*u3*(v1*v1)) + 2*u2*u3*u4*(v1*v1) -
             u3*(u4*u4)*(v1*v1) + u1*u2*u3*v1*v2 + u1*u2*u4*v1*v2 -
             u1*u3*u4*v1*v2 - 2*u2*u3*u4*v1*v2 - u1*(u4*u4)*v1*v2 +
             2*u3*(u4*u4)*v1*v2 - (u1*u1)*u4*(v2*v2) +
             u1*u3*u4*(v2*v2) + u1*(u4*u4)*(v2*v2) -
             u3*(u4*u4)*(v2*v2) + u1*(u2*u2)*v1*v3 -
             2*u1*u2*u4*v1*v3 + u1*(u4*u4)*v1*v3 - (u1*u1)*u2*v2*v3 +
             (u1*u1)*u4*v2*v3 + u1*u2*u4*v2*v3 - u1*(u4*u4)*v2*v3 -
             u1*(u2*u2)*v1*v4 - u1*u2*u3*v1*v4 + 2*(u2*u2)*u3*v1*v4 +
             u1*u2*u4*v1*v4 + u1*u3*u4*v1*v4 - 2*u2*u3*u4*v1*v4 +
             (u1*u1)*u2*v2*v4 - u1*u2*u3*v2*v4 + (u1*u1)*u4*v2*v4 -
             2*u1*u2*u4*v2*v4 - u1*u3*u4*v2*v4 + 2*u2*u3*u4*v2*v4 +
             (u1*u1)*u2*v3*v4 - u1*(u2*u2)*v3*v4 - (u1*u1)*u4*v3*v4 +
             u1*u2*u4*v3*v4 - (u1*u1)*u2*(v4*v4) +
             u1*(u2*u2)*(v4*v4) + u1*u2*u3*(v4*v4) -
             (u2*u2)*u3*(v4*v4))/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4)) -
            (u2*(u3*u3)*(v1*v1) - 2*u2*u3*u4*(v1*v1) +
             u2*(u4*u4)*(v1*v1) - u1*(u3*u3)*v1*v2 +
             2*u1*u3*u4*v1*v2 - u1*(u4*u4)*v1*v2 - u1*u2*u3*v1*v3 +
             u1*u2*u4*v1*v3 - u1*u3*u4*v1*v3 + 2*u2*u3*u4*v1*v3 +
             u1*(u4*u4)*v1*v3 - 2*u2*(u4*u4)*v1*v3 +
             (u1*u1)*u3*v2*v3 - (u1*u1)*u4*v2*v3 - u1*u3*u4*v2*v3 +
             u1*(u4*u4)*v2*v3 + (u1*u1)*u4*(v3*v3) -
             u1*u2*u4*(v3*v3) - u1*(u4*u4)*(v3*v3) +
             u2*(u4*u4)*(v3*v3) + u1*u2*u3*v1*v4 + u1*(u3*u3)*v1*v4 -
             2*u2*(u3*u3)*v1*v4 - u1*u2*u4*v1*v4 - u1*u3*u4*v1*v4 +
             2*u2*u3*u4*v1*v4 - (u1*u1)*u3*v2*v4 + u1*(u3*u3)*v2*v4 +
             (u1*u1)*u4*v2*v4 - u1*u3*u4*v2*v4 - (u1*u1)*u3*v3*v4 +
             u1*u2*u3*v3*v4 - (u1*u1)*u4*v3*v4 + u1*u2*u4*v3*v4 +
             2*u1*u3*u4*v3*v4 - 2*u2*u3*u4*v3*v4 + (u1*u1)*u3*(v4*v4) -
             u1*u2*u3*(v4*v4) - u1*(u3*u3)*(v4*v4) +
             u2*(u3*u3)*(v4*v4))/
            (u2*(u3*u3)*(v1*v1)*v2 - 2*u2*u3*u4*(v1*v1)*v2 +
             u2*(u4*u4)*(v1*v1)*v2 - u1*(u3*u3)*v1*(v2*v2) +
             2*u1*u3*u4*v1*(v2*v2) - u1*(u4*u4)*v1*(v2*v2) -
             (u2*u2)*u3*(v1*v1)*v3 + 2*u2*u3*u4*(v1*v1)*v3 -
             u3*(u4*u4)*(v1*v1)*v3 + 2*u1*u2*u4*v1*v2*v3 -
             2*u1*u3*u4*v1*v2*v3 - 2*u2*(u4*u4)*v1*v2*v3 +
             2*u3*(u4*u4)*v1*v2*v3 + (u1*u1)*u3*(v2*v2)*v3 -
             2*(u1*u1)*u4*(v2*v2)*v3 + 2*u1*(u4*u4)*(v2*v2)*v3 -
             u3*(u4*u4)*(v2*v2)*v3 + u1*(u2*u2)*v1*(v3*v3) -
             2*u1*u2*u4*v1*(v3*v3) + u1*(u4*u4)*v1*(v3*v3) -
             (u1*u1)*u2*v2*(v3*v3) + 2*(u1*u1)*u4*v2*(v3*v3) -
             2*u1*(u4*u4)*v2*(v3*v3) + u2*(u4*u4)*v2*(v3*v3) +
             2*(u2*u2)*u3*(v1*v1)*v4 - 2*u2*(u3*u3)*(v1*v1)*v4 -
             (u2*u2)*u4*(v1*v1)*v4 + (u3*u3)*u4*(v1*v1)*v4 -
             2*u1*u2*u3*v1*v2*v4 + 2*u1*(u3*u3)*v1*v2*v4 +
             2*u2*u3*u4*v1*v2*v4 - 2*(u3*u3)*u4*v1*v2*v4 +
             (u1*u1)*u4*(v2*v2)*v4 - 2*u1*u3*u4*(v2*v2)*v4 +
             (u3*u3)*u4*(v2*v2)*v4 - 2*u1*(u2*u2)*v1*v3*v4 +
             2*u1*u2*u3*v1*v3*v4 + 2*(u2*u2)*u4*v1*v3*v4 -
             2*u2*u3*u4*v1*v3*v4 + 2*(u1*u1)*u2*v2*v3*v4 -
             2*(u1*u1)*u3*v2*v3*v4 - 2*u1*u2*u4*v2*v3*v4 +
             2*u1*u3*u4*v2*v3*v4 - (u1*u1)*u4*(v3*v3)*v4 +
             2*u1*u2*u4*(v3*v3)*v4 - (u2*u2)*u4*(v3*v3)*v4 +
             u1*(u2*u2)*v1*(v4*v4) - 2*(u2*u2)*u3*v1*(v4*v4) -
             u1*(u3*u3)*v1*(v4*v4) + 2*u2*(u3*u3)*v1*(v4*v4) -
             (u1*u1)*u2*v2*(v4*v4) + 2*u1*u2*u3*v2*(v4*v4) -
             u2*(u3*u3)*v2*(v4*v4) + (u1*u1)*u3*v3*(v4*v4) -
             2*u1*u2*u3*v3*(v4*v4) + (u2*u2)*u3*v3*(v4*v4));


    /* put results into the output vector */
    rechirp_parameters[0] = a_inv;
    rechirp_parameters[1] = b_inv;
    rechirp_parameters[2] = c_inv;
    rechirp_parameters[3] = d_inv;
    rechirp_parameters[4] = e_inv;
    rechirp_parameters[5] = f_inv;
    rechirp_parameters[6] = g_inv;
    rechirp_parameters[7] = h_inv;
    return;         /* ?????? why is this return here??? not in book */
}

