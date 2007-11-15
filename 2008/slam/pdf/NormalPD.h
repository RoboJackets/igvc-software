#ifndef NORMAL_H
#define NORMAL_H

#include <flens/flens.h>
#include "PD.h"

using namespace flens;

// add functions for single dimention case

template<typename E>
class NormalPD {//: public PD<E> {
public:

	typedef DenseVector<Array<E> > MeanVector;
	//typedef SpMatrix<PackedStorage<E,ColMajor,Upper> > CovMatrix; // could use this if we wrote a SPMM blas routine
	typedef GeMatrix<FullStorage<E,ColMajor> > CovMatrix;

	/* Constructors */
	//NormalPD();
	NormalPD(MeanVector mu, CovMatrix sigma);

	int foo (void);
	/* Find the mean */
	MeanVector mean(void);

	/* Find the variance */
	CovMatrix var(void);

	/* Access a value from the probability density function */
	DenseVector<Array<Probability> > pdf(DenseVector<Array<E> > value);

	/* Access a value from the cumulative density function */
	DenseVector<Array<Probability> > cdf(DenseVector<Array<E> > value);

	/* Access an inverse value from the cumulative density function */
	DenseVector<Array<E> > icdf(DenseVector<Array<Probability> > prob);

	/* Set the mean and variance */
	void setMu(MeanVector mu);
	void setSigma(CovMatrix sigma);

private:
	MeanVector mu;
	CovMatrix sigma;
};

#include "NormalPD.tcc"

#if 0
template<typename E>
class NormalPDF : public NormalPD<E> { //, PDF {
public:
	typedef DenseVector<Array<E> > MeanVector;
	typedef SpMatrix<PackedStorage<E,ColMajor,Upper> > CovMatrix;


};

template<typename E>
class NormalCDF : public NormalPD<E> { //, CDF {
public:
	typedef DenseVector<Array<E> > MeanVector;
	typedef SpMatrix<PackedStorage<E,ColMajor,Upper> > CovMatrix;


};
#endif

#endif /* NORMAL_H */

