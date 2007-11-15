#ifndef PD_H
#define PD_H

#include <flens/flens.h>

using namespace flens;

// this should go somewhere else
typedef double Probability;	// should have a range of [0,1]

#if 0
enum {
	linear, //additive
	nonlinear // two ways to be nonlinear: x = A*y + z.^2 or x = A*y.*z
	//exponential
} linearity;
// Conditional
	//PD(int numConditionals, ...); // type, linearity of each conditional

/* Set the conditional relationship */


	// Conditional
	//Probability operator() (E value);
#endif

/**
 * Represents a general proablity distribution function.
 *	It can represent both continuous and discrete distributions. Continuous variables can
 *	either be linear or nonlinear.  Discrete distributions can ave either a constant sampling
 *	period or a variable one.
 *	It can handle marginal, conditional, and (hopefully in the future) joint probablities.
 *	
 */

// it would really cool to do this with a varatic template (will be part of C++0x),
// that way conditional types could be specified at compile time
// it would also be neat to add symbolic relationships (could be implemented with expression templates)


// Single or Multi-dimension?
template<typename E>
class PD {
public:
	typedef DenseVector<Array<E> > DEVector, MeanVector;
	//typedef SpMatrix<PackedStorage<E,ColMajor,Upper> > CovMatrix; // could use this if we wrote a SPMM blas routine
	typedef GeMatrix<FullStorage<E,ColMajor> > CovMatrix;

	/* Access a value from the probability density function */
	virtual Probability pdf(E value);

	/* Access a value from the cumulative density function */
	virtual Probability cdf(E value);

	/* Access an inverse value from the cumulative density function */
	virtual E icdf(Probability prob);

	/* Find the mean */
	virtual MeanVector mean(void); //maybe change this to expected value

	/* Find the variance */
	virtual CovMatrix var(void);

	/* Generate a signal base on the probablity distrubtion */
	virtual DEVector genSignal(int size);

	/**/
	//int dim(void);

private:
	//int dimension;

	/* Prevent instantiation */
	//PD(void);
};

#include "PD.tcc"

#endif /* PD_H */

