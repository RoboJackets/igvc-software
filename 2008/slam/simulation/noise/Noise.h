#ifndef NOISE_H
#define NOISE_H

#include <flens/flens.h>
#include <fftw3.h>

using namespace flens;
using namespace std;

template<typename E>
class Noise {
public:
	/* Constructor */
	Noise(int size, pdf<E> dist, fdf freq);

	//operator //return entire distrubtion
	operator() (int index); // return a single value

private:
	DenseVector<Array<E> > values;
};

#endif /* NOISE_H */

