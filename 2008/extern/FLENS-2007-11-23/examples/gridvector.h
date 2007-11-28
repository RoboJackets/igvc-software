#include <cmath>

namespace flens {

class GridVector;

template <>
struct TypeInfo<GridVector>
{
    typedef GridVector Impl;
    typedef double      ElementType;
};

class GridVector
    : public Vector<GridVector>
{
    public:
        typedef GeMatrix<FullStorage<double, RowMajor> >  Matrix;

        GridVector(Matrix &_grid)
            : grid(_grid)
        {
            assert(grid.firstRow()==0);
            assert(grid.firstCol()==0);
        }

        template <typename RHS>
        GridVector &
        operator=(const Vector<RHS> &rhs)
        {
            assign(rhs.impl(), *this);
            return *this;
        }

        template <typename RHS>
        GridVector &
        operator+=(const Vector<RHS> &rhs)
        {
            plusAssign(rhs.impl(), *this);
            return *this;
        }

        int     rh;
        Matrix  &grid;
};

double
norm(const GridVector &x)
{
    const GridVector::Matrix &X = x.grid;

    double r = 0;
    for (int i=X.firstRow(); i<=X.lastRow(); ++i) {
        for (int j=X.firstCol(); j<=X.lastCol(); ++j) {
            r += X(i,j)*X(i,j);
        }
    }
    return std::sqrt(r);
}

void
copy(const GridVector &x, GridVector &y)
{
    y.grid = x.grid;
}

void
scal(double alpha, GridVector &x)
{
    assert(0);
}


} // namespace flens












