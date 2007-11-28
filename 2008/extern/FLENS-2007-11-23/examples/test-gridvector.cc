#include <flens/flens.h>

namespace flens {

//-- Poisson2D -----------------------------------------------------------------

class Poisson2D;

template <>
struct TypeInfo<Poisson2D>
{
    typedef Poisson2D   Impl;
    typedef double      ElementType;
};

class Poisson2D
    : public SymmetricMatrix<Poisson2D>
{
    public:
        Poisson2D() {}

        Poisson2D(int _rh) : rh(_rh) {}

        int rh;
};

//-- GridVector2D --------------------------------------------------------------

class GridVector2D;

template <>
struct TypeInfo<GridVector2D>
{
    typedef GridVector2D Impl;
    typedef double       ElementType;
};

class GridVector2D
    : public Vector<GridVector2D>
{
    public:
        typedef GeMatrix<FullStorage<double, RowMajor> >  Grid;

        GridVector2D()
            : rh(0)
        {
        }


        GridVector2D(int _rh)
            : rh(_rh), grid(_(0,rh),_(0,rh))
        {
        }

        template <typename RHS>
        GridVector2D &
        operator=(const Vector<RHS> &rhs)
        {
            copy(rhs.impl(), *this);
            return *this;
        }

        template <typename RHS>
        GridVector2D &
        operator+=(const Vector<RHS> &rhs)
        {
            axpy(1., rhs.impl(), *this);
            return *this;
        }

        template <typename RHS>
        GridVector2D &
        operator-=(const Vector<RHS> &rhs)
        {
            axpy(-1., rhs.impl(), *this);
            return *this;
        }

        int    rh;
        Grid   grid;
};

double
dot(const GridVector2D &x, const GridVector2D &y)
{
    const GridVector2D::Grid &X = x.grid;
    const GridVector2D::Grid &Y = y.grid;

    assert(X.numRows()==Y.numRows());
    assert(X.numCols()==Y.numCols());

    int m = Y.lastRow()-1;
    int n = Y.lastCol()-1;

    double d=0;;
    for (int i=1; i<=m; ++i) {
        for (int j=1; j<=n; ++j) {
            d += X(i,j)*Y(i,j);
        }
    }
    return d;
}

void
copy(const GridVector2D &x, GridVector2D &y)
{
    y.grid = x.grid;
}

void
scal(double alpha, GridVector2D &x)
{
    x.grid *= alpha;
}

void
axpy(double alpha, const GridVector2D &x, GridVector2D &y)
{
    y.grid += alpha*x.grid;
}

template <typename ALPHA, typename BETA>
void
mv(ALPHA alpha, const Poisson2D &A, const GridVector2D &x,
   BETA beta, GridVector2D &y)
{
    assert(&x!=&y);

    const GridVector2D::Grid &X = x.grid;
    GridVector2D::Grid       &Y = y.grid;

    if ((Y.numRows()!=X.numRows()) || (Y.numCols()!=X.numCols())) {
        Y.resize(X.numRows(), X.numCols(), X.firstRow(), X.firstCol());
    }

    int m = Y.lastRow()-1;
    int n = Y.lastCol()-1;

    alpha *= A.rh*A.rh;
    for (int i=1; i<=m; ++i) {
        for (int j=1; j<=n; ++j) {
            Y(i,j)  = beta*Y(i,j);
            Y(i,j) += alpha*(4*X(i,j)-X(i-1,j)-X(i+1,j)-X(i,j-1)-X(i,j+1));
        }
    }
}

std::ofstream &
operator<<(std::ofstream &out, const GridVector2D &x)
{
    const GridVector2D::Grid &X = x.grid;

    out.precision(6);
    out.setf(std::ios::fixed);
    double h = 1./x.rh;
    for (int i=X.firstRow(); i<=X.lastRow(); ++i) {
        for (int j=X.firstCol(); j<=X.lastCol(); ++j) {
            double x = j*h;
            double y = i*h;
            out.width(12);
            out.operator<<(x);
            out << " ";
            out.operator<<(y);
            out << " ";
            out.operator<<(X(i,j));
            out << std::endl;
        }
        out << std::endl;
    }
    return out;
}

} // namespace flens

using namespace flens;
using namespace std;

int
main()
{
    int n = 1024;
    Poisson2D     A(n+1);
    GridVector2D  b(n+1), x(n+1);
    b.grid(_(1,n),_(1,n)) = 1;

    int it = cg(A, x, b);
    cout << "it = " << it << endl;

    ofstream file("poisson2d.dat");
    file << x << endl;
}

