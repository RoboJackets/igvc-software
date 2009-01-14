/*
   not yet for distribution (copyright notice, etc. and all that...

   Video orbits, "seamless hockney", "painting with looks"...
   assembling images by treating them like pieces of a jigsaw puzzle
   with overlapping pieces.  Matlab/fmex translated to c
   Steve Mann, 1989-1995

*/

/* interatively estimates the parameters of the phirp2 motion group */
/* to compile on a Sun: gcc -c -I/usr/openwin/include motion.c      */

#include "mat_util.h"
//#include "est_pseudo.h"
//#include "display_service.h"
#include "motion.h"
//#include "pchirp2ia.h"
#include <math.h>

#define MEAN_SQR_ERROR 0
#define CONTINUITY 1

/*this is define in main.c. It is a toggle switch*/
int NANINPUT;


/* ==================================================================== */
void pcompose(Var * A, Var * B, Var * Pout)
{
    /* - - realloc mem for 1 more element in A and B and Pout- - */
    A->data = (Data *) realloc(A->data, 9*sizeof(Data));
    B->data = (Data *) realloc(B->data, 9*sizeof(Data));
    Pout->data = (Data *) realloc(Pout->data, 9*sizeof(Data));

    /* - - change the matrix sizes - - */
    A->M = 3;
    B->M = 3;
    Pout->M = 3;
    A->N = 3;
    B->N = 3;
    Pout->N = 3;

    /* - - set last elements of A and B to 1 - - */
    A->data[8] = 1.0;
    B->data[8] = 1.0;
    Pout->data[8] = 0.0;

    /* - - multiply the matricies together - - */
    matrix_multiply(A, B, Pout);

    /* - - divid Pout by its last element if not too small - - */
    div_mat_by_constant(Pout, (Data) Pout->data[8], Pout);

    /* - - realloc A, B, Pout and change their sizes - - */
    A->data = (Data *) realloc(A->data, 8*sizeof(Data));
    B->data = (Data *) realloc(B->data, 8*sizeof(Data));
    Pout->data = (Data *) realloc(Pout->data, 8*sizeof(Data));
    A->M = 8;
    B->M = 8;
    Pout->M = 8;
    A->N = 1;
    B->N = 1;
    Pout->N = 1;
}


/* =================================================================== */
double calculate_difference(Var * A, Var * B, Var *ANS,
                            int diff_type, int USE_DISPLAY)
{
    double diff_index = 0;

    switch (diff_type)
    {
    case MEAN_SQR_ERROR:
    {
        sub_matrix(B, A, ANS);   /* ans in data fork */
        ANS->var_type = INTERNAL_VARIABLE;
        diff_index = meansquared( ANS );
        break;
    }
    case CONTINUITY:
    {
        /* additional difference methods to be added */
        break;
    }
    default:
    {
        break;
    }
    }

    /* - - Convert the diff_image to uchar if USE_DISPLAY - - */
    if (USE_DISPLAY)
    {
        absval(ANS);
        data_to_image(ANS);
        ANS->var_type = IMAGE;
    }

    return(diff_index);
}











