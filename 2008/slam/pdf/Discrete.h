#ifndef DISCRETE_H
#define DISCRETE_H

template<typename E>
class DiscreteBase : PDF {
	
}

/** Repesents a single- or multi-dimentional discrete probablity density function.
 *	It can handle discrete variables or sampled continous variables.
 *	It can handle a uniform or variable sampling period.
 */
template<typename E>
class DiscretePDF : PDF {
public:
	typedef DenseVector<Array<E> > DEVector;
	typedef DenseVector<Array<DEVector> > DDEVector;

	/* Constructors */
	DiscretePDF(int size); //DiscretePDF(DenseVector<Array<int> > size);

	DiscretePDF(DenseVector<Array<Probability> prob);

	DiscretePDF(E minVal, E maxVal, DenseVector<Array<Probability> prob);

	DiscretePDF(DEVector ranges, inclusivity = all, DenseVector<Array<Probability> prob);

	/* Access a value */
	Probability operator() (E value); //Probability operator() (DEVector value);

	/* Assign values */
	//operator= (Probability val, bool rescale = true)

	/* Convert  */
	//convert(DEVector values, bool scaleValues = true);
	//convert(DenseVector<Array<int> > size, DDEVector values);

	/* Change the size */
	//resize(int newSize);

private:
	DenseVector<Array<Probability> > bins;
	E stride;
	double scale;

	void rescale();
}

#endif /* DISCRETE_H */

