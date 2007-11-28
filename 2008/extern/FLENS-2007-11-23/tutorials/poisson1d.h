#include <flens/flens.h>

namespace flens {

//-- define matrix type Poisson1d ---------------------------------------

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
        Poisson1D(int _rh) : rh(_rh) {}
        int rh;
};

//-- implement matrix-vector product tailored for the Poisson1D matrix --

template <typename VX, typename VY>
void
mv(double alpha, const Poisson1D &A, const DenseVector<VX> &x,
   double beta, DenseVector<VY> &y)
{
    assert(&x!=&y);

    if (y.length()!=x.length()) {
        y.resize(x.length(), x.firstIndex());
    }

    int N = A.rh - 1;
    alpha *= (N+1)*(N+1);
    for (int i=1; i<=N; ++i) {
        y(i) = alpha*(-x(i-1) + 2*x(i) - x(i+1)) + beta*y(i);
    }
}

} // namespace flens
