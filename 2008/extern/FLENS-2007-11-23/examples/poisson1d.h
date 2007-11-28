#include <examples/gaussseidel.h>

namespace flens {

class Poisson1D;

template <>
struct TypeInfo<Poisson1D>
{
    typedef Poisson1D   Impl;
    typedef double      ElementType;
};

class Poisson1D
    : public SymmetricMatrix<Poisson1D>
{
    public:
        Poisson1D() {}

        Poisson1D(int _rh) : rh(_rh) {}

        int rh;
};

//----------------------------------------------------------------------------------------

template <typename ALPHA, typename VX, typename BETA, typename VY>
void
mv(ALPHA alpha, const Poisson1D &A, const DenseVector<VX> &x,
   BETA beta, DenseVector<VY> &y)
{
    assert(&x!=&y);

    if (y.length()!=x.length()) {
        y.resize(x.length(), x.firstIndex());
    }

    alpha *= A.rh*A.rh;
    int n = y.lastIndex()-1;

    for (int i=1; i<=n; ++i) {
        y(i) = alpha*(-x(i-1) + 2*x(i) -x(i+1)) + beta*y(i);
    }
}


template <typename VB, typename VX>
void
mv(const GaussSeidel<Poisson1D, DenseVector<VB> > &GS, DenseVector<VX> &x)
{
    assert(x.length()==GS.b.length());

    int    n         = x.lastIndex()-1;
    double omega     = GS.omega;
    double c         = omega/2.;
    double hh        = (1./GS.A.rh)*(1./GS.A.rh);

    for (int i=1; i<=n; ++i) {
        x(i) = (1-omega)*x(i) + c*(GS.b(i)*hh + x(i-1) + x(i+1));
    }
}

} // namespace flens












