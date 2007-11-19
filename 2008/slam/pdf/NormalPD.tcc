#include <flens/inv.h>

template<typename E> NormalPD<E>::NormalPD(MeanVector mu, CovMatrix sigma) :
	mu(mu), sigma(sigma) {
	assert(mu.length() == sigma.numRows());
}

template<typename E> typename NormalPD<E>::MeanVector NormalPD<E>::mean(void) {
	return(mu);
}

template<typename E> typename NormalPD<E>::CovMatrix NormalPD<E>::var(void) {
	return(sigma);
}
#if 0
template<typename E> DenseVector<Array<Probability> > NormalPD<E>::pdf(DenseVector<Array<E> > value) {
	return( );
}

template<typename E> DenseVector<Array<Probability> > NormalPD<E>::cdf(DenseVector<Array<E> > value) {
	return(  );
}

template<typename E> DenseVector<Array<E> > NormalPD<E>::icdf(DenseVector<Array<Probability> > prob) {
	return(  );
}

//single dimentional only
template<typename E> Probability NormalPD<E>::pdf(E value) {
	return( gsl_ran_gaussian_pdf((value + NormalPD<E>::mu),NormalPD<E>::sigma) );
}


template<typename E> Probability NormalPD<E>::cdf(E value) {
	return( gsl_cdf_gaussian_P((value + NormalPD<E>::mu),NormalPD<E>::sigma) );
}

template<typename E> E NormalPD<E>::icdf (Probability prob) {
	return( gsl_cdf_gaussian_Qinv(prob,NormalPD<E>::sigma) - NormalPD<E>::mu );
}
	// compute the pdf
	CovMatrix tempM(sigma.dim());
	MeanVector tempV1(mu.length);
	MeanVector tempV2(mu.length);
	E tempVal;

	tempM = inv(sigma);
	tempV1 = value - mu;
	tempV2 = tempM * tempV1;
	tempVal = tempV1 * tempV2;
	return ( (1/(pow((2*M_PI),(N/2))*sqrt(det(sigma)))) * exp(-tempVal/2) );
#endif

