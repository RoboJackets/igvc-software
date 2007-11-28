#include <cassert>
#include <flens/flens.h>
#include <iostream>

namespace flens {

class Richardson;

template <>
struct TypeInfo<Richardson>
{
    typedef Richardson Impl;
    typedef double     ElementType;
};

class Richardson
    : public SymmetricMatrix<Richardson>
{
    public:
        Richardson(int _dim, double _omega)
            : dim(_dim), omega(_omega)
        {
        }

        int    dim;
        double omega;
};



template <typename ALPHA, typename VX, typename BETA, typename VY>
void
mv(ALPHA alpha, const Richardson &A, const DenseVector<VX> &x,
   BETA beta, DenseVector<VY> &y)
{
    assert(x.length()==y.length());
    assert(alpha==ALPHA(1));
    assert(beta==BETA(0));

    int    n     = x.length();
    double omega = A.omega;

    y(1) = (1-omega*2)*x(1) + omega*x(2);
    for (int i=2; i<n; ++i) {
        y(i) = omega*x(i-1) + (1-omega*2)*x(i) + omega*x(i+1);
    }
    y(n) = omega*x(n-1) + (1-omega*2)*x(n);
}

} // namespace flens

using namespace flens;
using namespace std;

int
main()
{
    int    n = 10;
    double omega = .51;
    Richardson M(n, omega);

    DenseVector<Array<double> > x(n), b(n), xNew(n);

    for (int i=1; i<=n; ++i) {
        b(i) = 1;
    }

    for (int i=0; i<=800; ++i) {
        xNew = M*x + omega*b;
        x = xNew;
        cout << "x = " << x << endl;
    }
}












