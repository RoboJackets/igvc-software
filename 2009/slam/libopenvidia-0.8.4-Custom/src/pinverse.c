/*
   not yet for distribution (copyright notice, etc. and all that...

   Video orbits, "seamless hockney", "painting with looks"...
   assembling images by treating them like pieces of a jigsaw puzzle
   with overlapping pieces.  Matlab/fmex translated to c
   Steve Mann, 1989-1995

*/

/* pinverse.c  */

#include "mat_util.h"
#include <math.h>

/* ==================================================================== */

void pinverse(Var * P, Var * Pinverse)
{
    double a, b, c, d, e, f, g, h;
    double det;

    /* -- realloc mem for 1 more element in P and Pinverse -- */
    P->data = (Data *) realloc(P->data, 9*sizeof(Data));
    Pinverse->data = (Data *) realloc(Pinverse->data, 9*sizeof(Data));

    /* -- change the matrix sizes -- */
    P->M = 3;
    P->N = 3;
    Pinverse->M = 3;
    Pinverse->N = 3;

    /* -- set last elements of A to 1 -- */
    P->data[8] = 1.0;

    a=P->data[0];
    b=P->data[1];
    c=P->data[2];
    d=P->data[3];
    e=P->data[4];
    f=P->data[5];
    g=P->data[6];
    h=P->data[7];

    det = (a*e+b*f*g+d*h*c-g*e*c-a*h*f-d*b);

    /* fprintf(stderr, "det: %f\n", det); */

    Pinverse->data[0] = (e - f*h) / det;
    Pinverse->data[1] = (-b + c*h) / det;
    Pinverse->data[2] = (b*f - c*e) / det;
    Pinverse->data[3] = (-d + f*g) / det;
    Pinverse->data[4] = (a - c*g) / det;
    Pinverse->data[5] = (c*d - a*f) / det;
    Pinverse->data[6] = (d*h - e*g) / det;
    Pinverse->data[7] = (b*g - a*h) / det;
    Pinverse->data[8] = (a*e - b*d) / det;

#if 0
    Pinverse->data[0] =
        (- e + f*h)/det;
    Pinverse->data[1] =
        -  (- b + c*h)/(a*e+b*f*g+d*h*c-g*e*c-a*h*f-d*b);
    Pinverse->data[2] =
        -  (b*f - c*e)/(a*e+b*f*g+d*h*c-g*e*c-a*h*f-d*b);
    Pinverse->data[3] =
        -  (- d + f*g)/(a*e+b*f*g+d*h*c-g*e*c-a*h*f-d*b);
    Pinverse->data[4] =
        -  (a - c*g)/(a*e+b*f*g+d*h*c-g*e*c-a*h*f-d*b);
    Pinverse->data[5] =
        (a*f - c*d)/(a*e+b*f*g+d*h*c-g*e*c-a*h*f-d*b);
    Pinverse->data[6] =
        -  (d*h - e*g)/(a*e+b*f*g+d*h*c-g*e*c-a*h*f-d*b);
    Pinverse->data[7] =
        (a*h - b*g)/(a*e+b*f*g+d*h*c-g*e*c-a*h*f-d*b);
    Pinverse->data[8] =
        -  (a*e - d*b)/(a*e+b*f*g+d*h*c-g*e*c-a*h*f-d*b);
#endif

    /* printf("pinverse[2]=%f\n",Pinverse->data[1]);  */

    /*  fprintf(stderr, "Pinverse[8] = %f\n", Pinverse->data[8]); */
    /* -- divide Pinverse by its last element if not too small -- */
    div_mat_by_constant(Pinverse, (Data) Pinverse->data[8], Pinverse);

    /* -- realloc P, Pinverse, and change their sizes -- */
    Pinverse->data = (Data *) realloc(Pinverse->data, 8*sizeof(Data));
    P->M = 8;
    Pinverse->M = 8;
    P->N = 1;
    Pinverse->N = 1;
}

/* =================================================================== */

