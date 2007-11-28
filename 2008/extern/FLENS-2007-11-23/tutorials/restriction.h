#ifndef TUTORIALS_RESTRICTION_H
#define TUTORIALS_RESTRICTION_H 1

#include <tutorials/gridvector2d.h>

namespace flens {

class Restriction;

template <>
struct TypeInfo<Restriction>
{
    typedef Restriction Impl;
    typedef double      ElementType;
};

class Restriction
    : public GeneralMatrix<Restriction>
{
};

void
restrict(const GridVector2D &x, GridVector2D &y)
{
    const GridVector2D::Grid &X = x.grid;
    GridVector2D::Grid &Y = y.grid;

    int    m         = Y.lastRow()-1;
    int    n         = Y.lastCol()-1;

    for (int i=1, I=2; i<=m; ++i, I+=2) {
        for (int j=1, J=2; j<=n; ++j, J+=2) {
            Y(i,j) = (   X(I-1,J-1) + 2*X(I-1,J) +   X(I-1,J+1)
                     + 2*X(I  ,J-1) + 4*X(I  ,J) + 2*X(I  ,J+1)
                     +   X(I+1,J-1) + 2*X(I+1,J) +   X(I+1,J+1))/16;
        }
    }
}

void
mv(Transpose trans, double alpha, const Restriction &A, const GridVector2D &x,
   double beta, GridVector2D &y)
{
    assert(trans==NoTrans);
    assert(alpha==1.);
    assert(beta==0.);

    restrict(x.impl(), y.impl());
}

} // namespace flens

#endif // TUTORIALS_RESTRICTION_H
