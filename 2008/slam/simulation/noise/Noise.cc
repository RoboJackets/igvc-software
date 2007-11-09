#include "Noise.h"

/* This generates all the values at once. It might be faster to just generate them
 *	when they are requested.
 */
Noise::Noise(int size, pdf dist, fdf freq) :
	values(size) {

	/* Create an instance of a Tausworthe random number generator */
	gsl_rng *r = gsl_rng_alloc( gsl_rng_taus );

	/* Seed the generator */
	gsl_rng_set( r, (unsigned long int) time(NULL) );


	for(int i = 0; i < size; i++)  {
		//values.data[i] = gsl_cdf_gaussian_Pinv(gsl_rng_uniform (r), Sigma);
		values.data[i] = gsl_ran_gaussian(r, Sigma);
	}

	gsl_rng_free(r);
}
