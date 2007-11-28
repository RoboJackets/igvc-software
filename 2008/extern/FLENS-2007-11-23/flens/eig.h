#ifndef FLENS_EIG_H
#define FLENS_EIG_H 1

#include <flens/matvec.h>
#include <flens/resultclosures.h>

namespace flens {

template <typename MatrixType>
class eig_;

template <typename I>
    ResultClosure<eig_<I> >
    eig(const Matrix<I> &A);

} // namespace flens

#include <flens/eig.tcc>

#endif // FLENS_EIG_H












