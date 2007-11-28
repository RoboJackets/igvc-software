#ifndef SCIFI_GAUSSSEIDEL_H
#define SCIFI_GAUSSSEIDEL_H

#include <flens/matvec.h>

namespace flens {

template <typename MA, typename VB>
class GaussSeidel;

template <typename MA, typename VB>
struct TypeInfo<GaussSeidel<MA, VB> >
{
    typedef GaussSeidel<MA, VB> Impl;
    typedef double              ElementType;
};

template <typename MA, typename VB>
class GaussSeidel
    : public SymmetricMatrix<GaussSeidel<MA, VB> >
{
    public:
        typedef VB VectorType;

        GaussSeidel(const MA &_A, const VB &_b, double _omega)
            : A(_A), b(_b), omega(_omega)
        {
        }

        const MA  &A;
        const VB  &b;
        double    omega;
};

template <typename ALPHA, typename MA, typename VB,
          typename VX, typename BETA, typename VY>
void
mv(ALPHA alpha, const GaussSeidel<MA, VB> &A, const Vector<VX> &x,
   BETA beta, Vector<VY> &y)
{
    assert(alpha==ALPHA(1));
    assert(beta==BETA(0));
    assert(&x==&y);
    mv(A, y.impl());
}

} // namespace flens

#endif // SCIFI_GAUSSSEIDEL_H












