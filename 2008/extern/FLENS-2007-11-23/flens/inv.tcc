#include <flens/lapack.h>
#include <flens/array.h>
#include <flens/densevector.h>
#include <flens/lapack_flens.h>

namespace flens {

template <typename T>
GeMatrix<FullStorage<T, ColMajor> >
inv(const GeMatrix<FullStorage<T, ColMajor> > &A)
{
    GeMatrix<FullStorage<T, ColMajor> > IA = A;
    DenseVector<Array<int> > p(IA.numRows());
    trf(IA, p);
    tri(IA, p);
    return IA;
}

} // namespace












