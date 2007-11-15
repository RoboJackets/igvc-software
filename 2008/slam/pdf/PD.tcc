// use a Tausworthe random number generator
#define RNG gsl_rng_taus

#if 0
PD::virtual Probability pdf(E value) {
}

PD::virtual Probability cdf(E value) {
}

PD::virtual E icdf(Probability prob) {
}

PD::virtual MeanVector mean(void) {
	// Integrate the pdf over an appropriate range
}

PD::virtual CovMatrix var(void) {
}


PD::virtual DEVector genSignal(int size) {
	DEVector(size) values;

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

	return();
}
#endif

