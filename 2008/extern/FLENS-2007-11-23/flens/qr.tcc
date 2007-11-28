#include <flens/array.h>
#include <flens/densevector.h>
#include <flens/lapack_flens.h>
#include <flens/result.h>
#include <flens/solvertraits.h>

namespace flens {

template <typename T, StorageOrder Order>
void
_setupQ(GeMatrix<FullStorage<T, Order> > &Q,
        const GeMatrix<FullStorage<T, Order> > &QR,
        const DenseVector<Array<T> > &tau)
{
    orgqr(Q, tau);
}

template <typename MatrixType>
struct qr_
{
    typedef typename Result<Matrix<MatrixType> >::Type ResultMatrix;

    qr_(const typename TypeInfo<MatrixType>::Impl &A)
        : _A(A)
    {
    }

    // QR = qr(A)
    template <typename I>
    void
    operator()(GeMatrix<I> &QR) const
    {
        typedef typename GeMatrix<I>::ElementType T;
        DenseVector<Array<T> > tau;
        QR = _A;
        qrf(QR, tau);
    }

    // (Q, R) = qr(A)
    template <typename S1, typename S2>
    void
    operator()(Pair<GeMatrix<S1>, TrMatrix<S2> > &lhs) const
    {
        typedef typename GeMatrix<S1>::ElementType T;
        GeMatrix<S1> &Q = lhs.first;
        TrMatrix<S2> &R = lhs.second;
        DenseVector<Array<T> > tau;
        tau.resize(_(1, std::min(_A.numRows(),_A.numCols())));

        GeMatrix<S1> tmp = _A;
        tmp.shiftIndexTo(1,1);
        qrf(tmp, tau);

        R = upper(tmp); // CPOYING !!!!!!

        Q.resize(_A.numRows(), _A.numRows());
        int k = std::min(_A.numRows(), _A.numCols());
        Q(_,_(1,k)) = tmp(_,_(1,k));
        _setupQ(Q, tmp, tau);
    }


    private:
        typename SolverRefTrait<MatrixType, ResultMatrix>::Type _A;
};

template <typename I>
ResultClosure<qr_<I> >
qr(const Matrix<I> &A)
{
    return qr_<I>(A.impl());
}

} // namespace flens












