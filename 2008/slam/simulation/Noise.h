#ifndef NOISE_H
#define NOISE_H

#include <flens/flens.h>
#include "PDF.h"

using namespace flens;
using namespace std;

template<typename E>
class Noise {
public:
	/* Constructors */
	Noise(int size);
	Noise(int size, pdf<E> dist);
	Noise(int size, fdf freq);
	//Noise(int size, pdf<E> dist, fdf freq); //not possible

	/* Operators */
	E operator() (int index);	// return a single value
	DenseVector<Array<E> > operator() ();	// return all the values

private:
	DenseVector<Array<E> > values;
};

#endif /* NOISE_H */

