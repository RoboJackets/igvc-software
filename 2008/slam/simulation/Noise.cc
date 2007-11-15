#include "Noise.h"

// use a Tausworthe generator
#define RNG gsl_rng_taus

/* This class generates all the values at once. It might be faster to just generate them
 *	when they are requested.
 */

Noise::Noise(int size) :
	values(size) {

	/* Create an instance of a random number generator */
	gsl_rng *r = gsl_rng_alloc( RNG );

	/* Seed the generator */
	gsl_rng_set( r, (unsigned long int) time(NULL) );

	/* Generate the values */
	for(int i = 0; i < size; i++)  {
		//values(i) = gsl_cdf_gaussian_Pinv(gsl_rng_uniform (r), 1);
		values(i) = gsl_ran_gaussian(r, 1);
	}

	/* Free the generator */
	gsl_rng_free(r);
}

//XX
Noise::Noise(int size, pdf<E> dist) :
	values(size) {

	/* Create an instance of a random number generator */
	gsl_rng *r = gsl_rng_alloc( RNG );

	/* Seed the generator */
	gsl_rng_set( r, (unsigned long int) time(NULL) );

	/* Find the inverse CDF of the distrubution */
	icdf<E> distICDF(dist);

	/* Generate the values */
	for(int i = 0; i < size; i++)  {
		values(i) = distICDF(gsl_rng_uniform (r));
	}

	/* Free the generator */
	gsl_rng_free(r);
}


Noise::Noise(int size, DenseVector<Array<fftw_complex> > freq) :
	values(size) {
	/* Compute the inverse fourier transform of the distrubtion */
	fftw_plan p = fftw_plan_dft_c2r_1d(size, freq.data(), values.data(), FFTW_ESTIMATE);
	fftw_execute(p);
	fftw_destroy_plan(p);
}

E operator() (int index) {
	return( values(index) );
}

DenseVector<Array<E> > operator() () {
	return( values );
}

