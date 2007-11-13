#include "Normal.h"
#include <flens/inv.h>

//this is horrific

template<typename E> NormalPD<E>::NormalPD(MeanVector mu, CovMatrix sigma) :
	mu(mu), sigma(sigma) {
	assert(mu.length() == sigma.dim());
}

template<typename E> typename NormalPD<E>::MeanVector NormalPD<E>::mean() {
	return(mu);
}

template<typename E> typename NormalPD<E>::CovMatrix NormalPD<E>::var() {
	return(sigma);
}


template<typename E> Probability NormalPDF<E>::operator() (E value) {
	return( gsl_ran_gaussian_pdf((value + NormalPD<E>::mu),NormalPD<E>::sigma) );
}


template<typename E> Probability NormalCDF<E>::operator() (E value) {
	return( gsl_cdf_gaussian_P((value + NormalPD<E>::mu),NormalPD<E>::sigma) );
}

template<typename E> E NormalCDF<E>::operator() (Probability prob) {
	return( gsl_cdf_gaussian_Qinv(prob,NormalPD<E>::sigma) - NormalPD<E>::mu );
}

#if 0
	CovMatrix tempM(sigma.dim());
	MeanVector tempV1(mu.length);
	MeanVector tempV2(mu.length);
	E tempVal;

	tempM = inv(sigma);
	tempV1 = value - mu;
	tempV2 = tempM * tempV1;
	tempVal = tempV1 * tempV2;
	return ( (1/(pow((2*M_PI),(N/2))*sqrt(det(sigma)))) * exp(-tempVal/2) )
#endif

