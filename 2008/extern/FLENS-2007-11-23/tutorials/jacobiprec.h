#ifndef TUTORIALS_JACOBIPREC_H
#define TUTORIALS_JACOBIPREC_H 1

#include <flens/flens.h>

#include <tutorials/gridvector2d.h>
#include <tutorials/poisson2d.h>

namespace flens {

//-- define jacobi preconditioner as symmetric matrix -------------------

template <typename MA>
class JacobiPrec;

template <typename MA>
struct TypeInfo<JacobiPrec<MA> >
{
    typedef JacobiPrec<MA>  Impl;
    typedef double    ElementType;
};

template <typename MA>
class JacobiPrec
    : public SymmetricMatrix<JacobiPrec<MA> >
{
    public:
        JacobiPrec(const MA &_A)
            : A(_A)
        {
        }

        const MA  &A;
};

//-- muliplication is equivalent to a jacobi iteration ------------------
// x = JacobiPrec*b;   

void
mv(double alpha, const JacobiPrec<Poisson2D> &J, const GridVector2D &b,
   double beta, GridVector2D &x)
{
    assert(alpha==1.);
    assert(beta==0.);
    
    const GridVector2D::Grid &B = b.grid;
    GridVector2D::Grid       &X = x.grid;

    if ((X.firstRow()!=B.firstRow()) || (X.lastRow()!=B.lastRow())
     || (X.firstCol()!=B.firstCol()) || (X.lastCol()!=B.lastCol()))
    {
        X.resize(B.numRows(), B.numCols(), B.firstRow(), B.firstCol());
    }

    int    m   = B.lastRow()-1;
    int    n   = B.lastCol()-1;
    double hh  = 1./(J.A.rh*J.A.rh);
    for (int i=1; i<=m; ++i) {
        for (int j=1; j<=n; ++j) {
            X(i,j) = 0.25*hh*B(i,j);
        }
    }
}

} // namespace flens

#endif // TUTORIALS_JACOBIPREC_H
