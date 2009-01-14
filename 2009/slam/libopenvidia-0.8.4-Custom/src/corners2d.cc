/*
 ** Program: corners2d.c

 ** Used primarily to resample images containing periodicity in perspective
 ** so that the image itself contains the periodicity.
 ** Thus an image of a brick wall at an oblique angle may be remapped so
 ** that all the bricks appear the same size, and thus the ``chirping''
 ** phenomenon associated with perspective projection is eliminated.
 ** Chirping means that the spatial frequency changes with position in the
 ** image (one raster to a digital to analog converter, listened to by on
 ** a loudspeaker gives the bird-like or bat-like ``chirping'' sound).

 ** Dechirping may be thought of as moving all the vanishing points off to
 ** infinity. This program may be used directly to re-position the perspective,
 ** or may follow chirplet_est.c; parameters being estimated from peaks in the
 ** kurtosis of the Pan-Zoom (PZ) plane of the chirplet transform of the image.
 ** (Note, in this context the ``camera metaphor'' may be useful: for example
 ** the wavelet transform is a Translation, Zoom (TZ) plane.)

 ** Author: Steve Mann, 1992 July 9
 ** see also corners2r.c
 */

#include <math.h>
#include "corners2d.h"


/* Input Arguments */
#define CORNERS_IN      prhs[0]

/* Output Arguments */

#define RECHIRP_PARAMETERS      plhs[0]

#define max(A, B)       ((A) > (B) ? (A) : (B))
#define min(A, B)       ((A) < (B) ? (A) : (B))
#define pi 3.14159265

void corners2d(double dechirp_parameters[],
               double corners[])
{
    double x1,y1, x2,y2, x3,y3, x4,y4;
    double a,b,c,d,e,f,g,h;

    /* get corners from the input vector */
    x1 = corners[0];
    y1 = corners[1];
    x2 = corners[2];
    y2 = corners[3];
    x3 = corners[4];
    y3 = corners[5];
    x4 = corners[6];
    y4 = corners[7];

    /* DECCHIRP CODE IS HERE */
    /* the four corners are assumed to be on [0,1), not in pixel units */
    /* compute the 8 P-chirplet parameters, using letters of the alphabet,
       in order, starting with ``a'', as in the wavelet: (a,b,...) */
    c =  x1;
    f =  y1;   /* Emacs: control space (Mark set) then goto end of eq'n
                    then Meta-x indent-region */
    a = -((x2*x3 - x3*x4)*(-f + y2)/
          (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4)) -
        (x2*x3 - x3*x4)*(-f + y3)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-(x2*x3) + x3*x4)*(-f + y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x3)*(-(x4*y2) - x2*y3 + x4*y3 + x2*y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x4)*(x3*y2 - x3*y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x2)*(-(x3*y2) + x3*y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4);

    b = -((-(x2*x3) + x2*x4)*(-f + y2)/
          (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4)) -
        (-(x2*x3) + x2*x4)*(-f + y3)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (x2*x3 - x2*x4)*(-f + y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x3)*(x2*y3 - x2*y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x4)*(-(x2*y3) + x2*y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x2)*(x3*y2 - x4*y2 + x4*y3 - x3*y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4);

    d = -((-f + y2)*(x2*y3 - x4*y3)/
          (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4)) -
        (-(x2*y3) + x4*y3)*(-f + y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-f + y3)*(x3*y2 - x4*y2 + x2*y4 - x3*y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x4)*(y2*y3 - y3*y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x2)*(-(y2*y3) + y3*y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x3)*(-(y2*y3) + y3*y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4);

    e = -((-(x3*y2) + x4*y2)*(-f + y3)/
          (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4)) -
        (x3*y2 - x4*y2)*(-f + y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-f + y2)*(-(x2*y3) + x4*y3 + x2*y4 - x3*y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x2)*(y2*y3 - y2*y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x3)*(y2*y3 - y2*y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x4)*(-(y2*y3) + y2*y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4);

    g = -((x2 - x4)*(-f + y2)/
          (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4)) -
        (x2 - x4)*(-f + y3)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x4)*(y2 - y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-x2 + x4)*(-f + y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x2)*(-y2 + y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x3)*(-y2 + y4)/(-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4);

    h = -((-x3 + x4)*(-f + y2)/
          (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4)) -
        (-x3 + x4)*(-f + y3)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x2)*(y3 - y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x3)*(y3 - y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (x3 - x4)*(-f + y4)/
        (-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4) -
        (-c + x4)*(-y3 + y4)/(-(x3*y2) + x4*y2 + x2*y3 - x4*y3 - x2*y4 + x3*y4);


    /* put results into the output vector */
    dechirp_parameters[0] = a;
    dechirp_parameters[1] = b;
    dechirp_parameters[2] = c;
    dechirp_parameters[3] = d;
    dechirp_parameters[4] = e;
    dechirp_parameters[5] = f;
    dechirp_parameters[6] = g;
    dechirp_parameters[7] = h;
    return;         /* ?????? why is this return here??? not in book */
}

