#ifndef FLENS_QR_H
#define FLENS_QR_H 1

#include <flens/matvec.h>
#include <flens/matvecclosures.h>
#include <flens/resultclosures.h>

namespace flens {

template <typename MatrixType>
    class qr_;

template <typename I>
    ResultClosure<qr_<I> >
    qr(const Matrix<I> &A);
}

#include <flens/qr.tcc>

#endif // FLENS_QR_H












