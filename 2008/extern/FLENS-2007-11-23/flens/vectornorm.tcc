#include <numeric>
#include <flens/blas_flens.h>

namespace flens {

template <typename T>
struct VectorNorm<l1, T>
{
    static const typename NormTraits<T>::Type
    apply(const DenseVector<Array<T> > &x)
    {
        return asum(x);
    }

    static const typename NormTraits<T>::Type
    apply(const Array<T> &x)
    {
        return std::accumulate(x.data(), x.data()+x.length(), T(0));
    }
};

template <typename T>
struct VectorNorm<l2, T>
{
    static const typename NormTraits<T>::Type
    apply(const DenseVector<Array<T> > &x)
    {
        return nrm2(x);
    }
};

template <typename T>
struct VectorNorm<lInfinity, T>
{
    static const typename NormTraits<T>::Type
    apply(const DenseVector<Array<T> > &x)
    {
        return std::abs(x(amax(x)));
    }
};

template <NormType normType, typename T>
typename NormTraits<T>::Type
norm(const DenseVector<Array<T> > &x)
{
    return VectorNorm<normType, T>::apply(x);
}

template <NormType normType, typename T>
typename NormTraits<T>::Type
norm(const Array<T> &x)
{
    return VectorNorm<normType, T>::apply(x);
}

} // namespace flens












