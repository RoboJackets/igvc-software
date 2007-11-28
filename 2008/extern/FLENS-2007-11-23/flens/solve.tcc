#include <flens/blas.h>
#include <flens/blas_flens.h>
#include <flens/lapack_flens.h>
#include <flens/qr.h>
#include <flens/result.h>
#include <flens/solvertraits.h>
#include <flens/traits.h>

namespace flens {

template <typename T, StorageOrder Order>
void
solve(DenseVector<Array<T> > &xb,
      Transpose conjTrans,
      const GeMatrix<FullStorage<T, Order> > &A)
{
    if (A.numRows()==A.numCols()) {
        GeMatrix<FullStorage<T, Order> > Tmp = A;
        DenseVector<Array<int> > p(A.numRows());
        trf(Tmp, p);
        trs(conjTrans, Tmp, p, xb);
    } else {
        GeMatrix<FullStorage<T, ColMajor> > Q;
        TrMatrix<FullStorage<T, ColMajor> > R;
        (Q, R) = qr(A);
        xb = transpose(Q)*xb / R;
    }
}


template <typename E, typename S>
void
solve(DenseVector<E> &xb,
      Transpose conjTrans,
      const TrMatrix<S> &A)
{
    trs(conjTrans, A.impl(), xb);
}

template <typename T, StorageOrder Order>
void
solve(GeMatrix<FullStorage<T, Order> > &XB,
      Transpose conjTrans,
      const TrMatrix<ConstFullStorageView<T, Order> > &A)
{
    trsm(Left, conjTrans, T(1), A, XB);
}
/*
template <typename T, StorageOrder Order>
void
solve(GeneralMatrix<FullStorage<T, Order> > &XB,
      Transpose conjTrans,
      const GeneralMatrix<FullStorage<T, Order> > &A)
{
    if (A.numRows()==A.numCols()) {
        GeneralMatrix<FullStorage<T, Order> > Tmp = A;
        DenseVector<long> p(A.numRows());
        getrf(Tmp, p);
        getrs(conjTrans, Tmp, p, XB);
    } else {
        GeneralMatrix<FullStorage<double, ColMajor> > Q;
        TriangularMatrix<FullStorage<double, ColMajor> > R;
        (Q,R) = qr(A);
        XB = transpose(Q)*XB / R;
    }
}
*/
template <typename B, typename A>
struct solve_
{
    solve_(const typename TypeInfo<B>::Impl &b,
           const typename TypeInfo<A>::Impl &a)
        : _b(b), _a(a)
    {
    }

    template <typename LHS>
    void
    operator()(LHS &lhs) const
    {
        lhs = _b;
        solve(lhs, NoTrans, _a.impl());
    }

    private:
        typedef typename Result<A>::Type ResA;

        const typename TypeInfo<B>::Impl &_b;
        typename SolverRefTrait<A, ResA>::Type _a;
};

template <typename V, typename M>
ResultClosure<solve_<typename V::Impl, typename M::Impl> >
operator/(const Vector<V> &b, const Matrix<M> &A)
{
    return solve_<typename V::Impl, typename M::Impl>(b.impl(), A.impl());
}
/*
template <typename M1, typename M2>
ResultClosure<solve_<M1, M2> >
operator/(const Matrix<M1> &B, const Matrix<M2> &A)
{
    return solve_<M1, M2>(B.impl(), A.impl());
}
*/
template <typename I, typename B, typename A>
void
assign(Vector<I> &lhs,
       const ResultClosure<solve_<B,A> > &rhs,
       typename Vector<I>::ElementType alpha,
       typename Vector<I>::ElementType beta)
{
    rhs.assignTo(lhs.impl());
}
/*
template <typename I, typename B, typename A>
void
assign(Matrix<I> &lhs,
       const ResultClosure<solve_<B,A> > &rhs,
       Transpose conjTrans,
       typename Matrix<I>::ElementType alpha,
       typename Matrix<I>::ElementType beta)
{
    rhs.assignTo(lhs.impl());
}
*/

} // namespace flens












