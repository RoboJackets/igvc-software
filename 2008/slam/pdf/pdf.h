#ifndef PDF_H
#define PDF_H

typedef Probability double;	// range [0,1]

enum {
	discrete,
	contLinear,
	contNonlinear,
} Linearity;

/**
 * Represents a general proablity density function.
 *	It can handle marginal, conditional, and (hopefully in the future) joint probablities.
 *	It can represent both continuous and discrete distributions. Continuous variables can
 *	either be linear or nonlinear.  Discrete distributions can ave either a constant sampling
 *	period or a variable one.
 *	
 */

// it would really cool to do this with a varatic template (will be part of C++0x)
// that would allow multiple types
template<typename E>
class pdf {
public:
	/* Constructors */
	pdf(int size, Linearity lin, int numConditionals = 0, ...);

	/* Set the */
	set(...);

	/* Access a value */
	Probability operator() (...);

	/* Access all the values */
	pdf<E> operator() (...);

private:
}

#endif /* PDF_H */

