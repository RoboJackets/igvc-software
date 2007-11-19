#ifndef PD_H
#define PD_H

#include <flens/flens.h>

using namespace flens;

// this should go somewhere else
typedef double Probability;	// range of [0,1]

/**
 * Abstract representation of a marginal proablity distribution.
 *	
 */

// it would really cool to do this with a varatic template (will be part of C++0x),
// that way conditional types could be specified at compile time

// it would also be nicer to make a GEMatrix a template typedef (also part of C++0x)

// should add functions for single dimention case
template<typename E>
class PD {
public:
	typedef DenseVector<Array<E> > DEVector, MeanVector;
	//typedef SpMatrix<PackedStorage<E,ColMajor,Upper> > CovMatrix; // could use this if we wrote a SPMM blas routine
	typedef GeMatrix<FullStorage<E,ColMajor> > CovMatrix;

	/* Access a value from the probability density function */
	virtual DenseVector<Array<Probability> > pdf(DenseVector<Array<E> > value) = 0;

	/* Access a value from the cumulative density function */
	virtual DenseVector<Array<Probability> > cdf(DenseVector<Array<E> > value) = 0;

	/* Access an inverse value from the cumulative density function */
	virtual DenseVector<Array<E> > icdf(DenseVector<Array<Probability> > prob) = 0;

	/* Find the mean */
	virtual MeanVector mean(void) = 0; //maybe change this to expected value

	/* Find the variance */
	virtual CovMatrix var(void) = 0;

	/* Generate a signal base on the probablity distrubtion */
	virtual DEVector genSignal(int size);

	/**/
	//int dim(void);

private:
	//int dimension;

	/* Prevent instantiation */
	PD(void);
};

#include "PD.tcc"

#endif /* PD_H */

