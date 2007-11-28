#ifndef FLENS_INV_H
#define FLENS_INV_H 1

#include <flens/generalmatrix.h>
#include <flens/matvec.h>
#include <flens/resultclosures.h>

namespace flens {

// TODO: rewrite inv(...) returning a ResultClosure.
template <typename T>
    GeMatrix<FullStorage<T,ColMajor> >
    inv(const GeMatrix<FullStorage<T,ColMajor> > &A) __attribute__((deprecated));

} // namespace flens

#include <flens/inv.tcc>

#endif // FLENS_INV_H












