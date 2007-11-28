#ifndef TUTORIALS_SMOOTHERGS_H
#define TUTORIALS_SMOOTHERGS_H 1

#include <tutorials/poisson2d.h>
#include <tutorials/gridvector2d.h>
#include <tutorials/gridvector2d_blas.h>

namespace flens {

template <typename MA, typename VB>
class SmootherGS;

template <typename MA,  typename VB>
struct TypeInfo<SmootherGS<MA, VB> >
{
    typedef SmootherGS<MA, VB> Impl;
    typedef double             ElementType;
};

template <typename MA, typename VB>
class SmootherGS
    : public GeneralMatrix<SmootherGS<MA, VB> >
{
    public:
        SmootherGS(const MA &_A, const VB &_b)
            : A(_A), b(_b)
        {
        }

        const MA  &A;
        const VB  &b;
};

// compute: x = SmootherGS*x;
void
gaussSeidel(const SmootherGS<Poisson2D, GridVector2D> &GS, GridVector2D &x)
{
    const GridVector2D::Grid &B = GS.b.grid;
    GridVector2D::Grid       &X = x.grid;

    int    m         = B.lastRow()-1;
    int    n         = B.lastCol()-1;
    double hh        = 1./(GS.A.rh*GS.A.rh);

    for (int i=1; i<=m; ++i) {
        for (int j=1; j<=n; ++j) {
            X(i,j) = 0.25*(B(i,j)*hh + X(i-1,j) + X(i+1,j) + X(i,j-1) + X(i,j+1));
        }
    }
}

// wrapper for: x = SmootherGS*x;
void
mv(Transpose trans, double alpha, const SmootherGS<Poisson2D, GridVector2D> &GS,
   const GridVector2D &b, double beta, GridVector2D &x)
{
    assert(trans==NoTrans);
    assert(alpha==1.);
    assert(beta==0.);
    assert(&b==&x);
    gaussSeidel(GS, x);
}

} // namespace flens

#endif // TUTORIALS_SMOOTHERGS_H
