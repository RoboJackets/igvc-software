#include <cassert>
#include <complex>

#include <flens/array.h>
#include <flens/densevector.h>
#include <flens/generalmatrix.h>
#include <flens/fullstorage.h>
#include <flens/lapack_flens.h>
#include <flens/result.h>
#include <flens/solvertraits.h>

namespace flens {

template <typename T>
void
eig(DenseVector<Array<T> > &wr,
    DenseVector<Array<T> > &wi,
    const GeMatrix<FullStorage<T, ColMajor> > &A)
{
    assert(A.numRows()==A.numCols());

    typedef FullStorage<T, ColMajor> Storage;
    typedef GeMatrix<Storage> GeMat;

    GeMat tmp = A;
    GeMat empty;

    wr.resize(A.numRows());
    wi.resize(A.numRows());
    ev(false, false, tmp, wr, wi, empty, empty);
}

template <typename T>
void
eig(DenseVector<Array<T> > &wr,
    DenseVector<Array<T> > &wi,
    GeMatrix<FullStorage<T, ColMajor> > &VR,
    const GeMatrix<FullStorage<T, ColMajor> > &A)
{
    assert(A.numRows()==A.numCols());

    typedef FullStorage<T, ColMajor> Storage;
    typedef GeMatrix<Storage> GeMat;

    GeMat tmp = A.impl(), empty;
    VR.resize(A.numRows(),A.numRows());

    wr.resize(A.numRows());
    wi.resize(A.numRows());
    ev(false, true, tmp, wr, wi, empty, VR);
}

template <typename MatrixType>
struct eig_
{
    eig_(const typename TypeInfo<MatrixType>::Impl &A)
        : _A(A)
    {
    }

    // (wr, wi) = eig(A)
    template <typename T>
    void
    operator()(Pair<DenseVector<Array<T> >,
                    DenseVector<Array<T> > > &lhs) const
    {
        eig(lhs.first, lhs.second, _A);
    }

    // (wr, wi, VR) = eig(A)
    template <typename T, typename S>
    void
    operator()(Triple<DenseVector<Array<T> >,
                      DenseVector<Array<T> >,
                      GeMatrix<S> > &lhs) const
    {
        eig(lhs.first, lhs.second, lhs.third, _A);
    }


    // d = eig(A) {complex}
    template <typename T>
    void
    operator()(DenseVector<Array<std::complex<T> > > &d) const
    {
        DenseVector<Array<T> > re, imag;
        eig(re, imag, _A);
        d.resize(re.range());
        for (int i=d.firstIndex(); i<=d.lastIndex(); ++i) {
            d(i) = std::complex<T>(re(i),imag(i));
        }
    }

    // d = eig(A) {double}
    void
    operator()(DenseVector<Array<double> > &d) const
    {
        DenseVector<Array<double> > imag;
        eig(d, imag, _A);
        #ifdef DEBUG
        for (int i=imag.firstIndex(); i<=imag.lastIndex(); ++i) {
            assert(fabs(imag(i)<=1e-15));
        }
        #endif
    }

    // (d, V) = eig(A)
    template <typename S, typename T>
    void
    operator()(Pair<DenseVector<Array<T> >, GeMatrix<S> > &lhs) const
    {
        eig(lhs.first, lhs.second, _A);
    }

    private:
        typedef typename Result<Matrix<MatrixType> >::Type ResultType;
        typename SolverRefTrait<MatrixType, ResultType>::Type _A;
};

template <typename I>
ResultClosure<eig_<I> >
eig(const Matrix<I> &A)
{
    return eig_<I>(A.impl());
}

} // namespace flens













