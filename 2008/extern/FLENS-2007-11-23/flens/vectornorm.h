#ifndef FLENS_VECTORNORM_H
#define FLENS_VECTORNORM_H 1

#include <flens/array.h>
#include <flens/matvec.h>

namespace flens {

enum NormType { l1, l2, lInfinity, Frobenius};

// struct NormTraits to be specialized for T=int and l2 e.g.
template <typename T>
struct NormTraits
{
    typedef T Type;
};

template <NormType normType, typename T>
struct VectorNorm
{
};

template <NormType normType, typename T>
    typename NormTraits<T>::Type
    norm(const DenseVector<Array<T> > &x);

template <NormType normType, typename T>
    typename NormTraits<T>::Type
    norm(const Array<T> &x);

} // namespace flens

#include <flens/vectornorm.tcc>

#endif // FLENS_VECTORNORM_H












