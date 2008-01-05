#ifndef DISCRETE_H
#define DISCRETE_H

//static getDiscretePDfromPDF

/** Repesents a single- or multi-dimentional discrete probablity density function.
 *	It can handle discrete variables or sampled continous variables.
 *	It can handle a uniform or variable sampling period.
 */

// single dim only
template<typename E>
class DiscretePD {
public:
	typedef DenseVector<Array<E> > DEVector;
	typedef DenseVector<Array<DEVector> > DDEVector;

	/* Constructors */
	DiscretePD(int size); //DiscretePD(DenseVector<Array<int> > size);

	DiscretePD(DenseVector<Array<Probability> prob);

	DiscretePD(E minVal, E maxVal, DenseVector<Array<E> value);

	DiscretePD(DEVector ranges, DenseVector<Array<E> value);

	/* Access a value */
	Probability operator() (E value); //Probability operator() (DEVector value);

	/* Assign values */
	//operator= (Probability val, bool rescale = true)

	/* Change the size */
	//resize(int newSize);

private:


	//void rescale();
}

#include "Discrete.tcc"

#endif /* DISCRETE_H */

