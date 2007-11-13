#ifndef PDF_H
#define PDF_H

typedef double Probability;	// should have a range of [0,1]

#if 0
enum {
	linear, //additive
	nonlinear // two ways to be nonlinear: x = A*y + z.^2 or x = A*y.*z
	//exponential
} linearity;
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
template<typename E>
class PDF {
public:
	typedef DenseVector<Array<E> > MeanVector;
	typedef SpMatrix<PackedStorage<E,ColMajor,Upper> > CovMatrix;

	/* Constructors */
	PDF(CDF dist);

	// Marginal
	PDF(Probablity (*func)(E value));

	// Conditional
	PDF(int numConditionals, ...); // type, linearity of each conditional

	/* Set the conditional relationship */

	/* Assignment */
	//operator= ();

	/* Access a value */

	// Marginal
	Probability operator() (E value);

	// Conditional
	Probability operator() (E value);

	/* Find the mean */
	virtual MeanVector mean();

	/* Find the variance */
	virtual CovMatrix var();

private:
}

#endif /* PDF_H */

