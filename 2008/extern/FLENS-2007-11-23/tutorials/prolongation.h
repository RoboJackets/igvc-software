#ifndef TUTORIALS_PROLONGATION_H
#define TUTORIALS_PROLONGATION_H 1

#include <tutorials/gridvector2d.h>

namespace flens {

class Prolongation;

template <>
struct TypeInfo<Prolongation>
{
    typedef Prolongation Impl;
    typedef double       ElementType;
};

class Prolongation
    : public GeneralMatrix<Prolongation>
{
};

void
prologate(const GridVector2D &x, GridVector2D &y)
{
    const GridVector2D::Grid &X = x.grid;
    GridVector2D::Grid &Y = y.grid;

    int    m         = X.lastRow()-1;
    int    n         = X.lastCol()-1;

    for (int i=1, I=2; i<=m; ++i, I+=2) {
        for (int j=1, J=2; j<=n; ++j, J+=2) {
            Y(I-1,J-1)+=.25*X(i,j); Y(I-1,J)+=.5*X(i,j); Y(I-1,J+1)+=.25*X(i,j);
            Y(I  ,J-1)+=.50*X(i,j); Y(I  ,J)+=   X(i,j); Y(I  ,J+1)+=.50*X(i,j);
            Y(I+1,J-1)+=.25*X(i,j); Y(I+1,J)+=.5*X(i,j); Y(I+1,J+1)+=.25*X(i,j);
        }
    }
}

void
mv(Transpose trans, double alpha, const Prolongation &A, const GridVector2D &x,
   double beta, GridVector2D &y)
{
    assert(trans==NoTrans);
    assert(alpha==1.);
    assert(beta==1.);

    prologate(x.impl(), y.impl());
}

} // namespace flens

#endif // TUTORIALS_PROLONGATION_H
