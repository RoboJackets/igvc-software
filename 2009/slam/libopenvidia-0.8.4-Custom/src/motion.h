#ifndef _MOTION_H_
#define _MOTION_H_

typedef struct {
    int NO_TRANSLATION;
    int NO_ZOOM;
    /* Following two options are mutually exclusive */
    int YES_ISOTROPIC;   /* Forces X and Y zoom to be the same */
    int ONLY_ROTATION;   /* Constrain A matrix to be rotation matrix */
    int NO_SHEAR;        /* No rotation at all */
    int NO_CHIRP;
} EstimatorParameters; /* Note that all-zero structure yields
			  the original behavior of the estimator */

void est_pchirp2(Var *, Var *, Var *);
void pcompose(Var *, Var *, Var *);
double calculate_difference(Var *, Var *, Var *, int, int);

#endif
