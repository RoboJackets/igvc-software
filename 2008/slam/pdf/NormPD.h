#ifndef NORMPD_H
#define NORMPD_H

#include <flens/flens.h>

using namespace flens;

/*
 * Normal (Gaussian) probablity distribution storage class.
 */
template <X>
class NormPD {
public:

	typedef SpMatrix<PackedStorage<E,ColMajor,Upper>> CovMatrix;
	typedef DenseVector<Array<X>> MeanVector;

	NormPD(int size);
	NormPD(MeanVector mu, CovMatrix Sigma);

	MeanVector mu;
	CovMatrix Sigma;
}

#endif /* NORMPD_H */

