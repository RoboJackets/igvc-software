#include <gsl/gsl_rng.h>

// use a Tausworthe random number generator
#define RNG gsl_rng_taus

#if 0
template<typename E> typename PD::MeanVector PD::mean(void) {
	// Integrate the pdf over an appropriate range
}

template<typename E> typename PD::CovMatrix PD::var(void) {
}
#endif

template<typename E> typename PD<E>::DEVector PD<E>::genSignal(int size) {
	DEVector values(size);

	/* Create an instance of a random number generator */
	gsl_rng *r = gsl_rng_alloc( RNG );

	/* Seed the generator */
	gsl_rng_set( r, (unsigned long int) time(NULL) );

	/* Generate the values */
	for(int i = 0; i < size; i++)  {
		values(i) = icdf(gsl_rng_uniform (r));
	}

	/* Free the generator */
	gsl_rng_free(r);

	return(values);
}

