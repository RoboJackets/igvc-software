#ifndef NORMAL_H
#define NORMAL_H

#include <flens/flens.h>
//#include "PDF.h"

using namespace flens;

typedef double Probability;	//should go somewhere else

template<typename E>
class NormalPD {
public:

	typedef DenseVector<Array<E> > MeanVector;
	typedef SpMatrix<PackedStorage<E,ColMajor,Upper> > CovMatrix;

	/* Constructors */
	NormalPD(MeanVector mu, CovMatrix sigma);
	//NormalPD(NormalPD dist); //is this the same as the default copy constructor?

	/* Find the mean */
	MeanVector mean();

	/* Find the variance */
	CovMatrix var();

private:
	MeanVector mu;
	CovMatrix sigma;
};


template<typename E>
class NormalPDF : public NormalPD<E> { //, PDF {
public:
	typedef DenseVector<Array<E> > MeanVector;
	typedef SpMatrix<PackedStorage<E,ColMajor,Upper> > CovMatrix;

	/* Access a value */
	Probability operator() (E value);
};

template<typename E>
class NormalCDF : public NormalPD<E> { //, CDF {
public:
	typedef DenseVector<Array<E> > MeanVector;
	typedef SpMatrix<PackedStorage<E,ColMajor,Upper> > CovMatrix;

	/* Access a value */
	Probability operator() (E value);

	/* Access an inverse value */
	E operator() (Probability prob);
};

#endif /* NORMAL_H */

