#ifndef FLENS_SOLVE_H
#define FLENS_SOLVE_H 1

#include <flens/densevector.h>
#include <flens/matvec.h>
#include <flens/resultclosures.h>
#include <flens/triangularmatrix.h>

namespace flens {

template <typename T, StorageOrder Order>
    void
    solve(DenseVector<Array<T> > &xb,
          Transpose conjTrans,
          const GeMatrix<FullStorage<T, Order> > &A);

template <typename E, typename S>
    void
    solve(DenseVector<E> &x,
          Transpose conjTrans,
          const TrMatrix<S> &A);


template <typename T, StorageOrder Order>
    void
    solve(GeMatrix<FullStorage<T, Order> > &XB,
          Transpose conjTrans,
          const TrMatrix<ConstFullStorageView<T, Order> > &A);
/*
template <typename T, StorageOrder Order>
    void
    solve(GeneralMatrix<FullStorage<T, Order> > &XB,
          Transpose conjTrans,
          const GeneralMatrix<FullStorage<T, Order> > &A);
*/
//------------------------------------------------------------------------------

template <typename B, typename A>
    class solve_;

template <typename V, typename M>
    ResultClosure<solve_<typename V::Impl, typename M::Impl> >
    operator/(const Vector<V> &b, const Matrix<M> &A);

template <typename M1, typename M2>
    ResultClosure<solve_<M1, M2> >
    operator/(const Matrix<M1> &B, const Matrix<M2> &A);

} // namespace flens

#include <flens/solve.tcc>

#endif // FLENS_SOLVE_H












