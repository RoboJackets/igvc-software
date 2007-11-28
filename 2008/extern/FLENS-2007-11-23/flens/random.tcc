#include <cstdlib>

#include <flens/densevector.h>
#include <flens/generalmatrix.h>

namespace flens {

template <Distribution dist, typename T>
void
random(DenseVector<Array<T> > &x)
{
    for (int i=x.firstIndex(); i<=x.lastIndex(); ++i) {
        x(i) = randomValue<dist, T>();
    }
}

template <Distribution dist, typename T>
void
random(DenseVector<Array<T> > &x, int length)
{
    x.resize(length);
    random<dist, T>(x);
}

template <Distribution dist, typename T, StorageOrder Order>
void
random(GeMatrix<FullStorage<T, Order> > &A)
{
    for (int i=A.firstRow(); i<=A.lastRow(); ++i) {
        for (int j=A.firstCol(); j<=A.lastCol(); ++j) {
            A(i,j) = randomValue<dist, T>();
        }
    }
}

template <Distribution dist, typename T, StorageOrder Order>
void
random(GeMatrix<FullStorage<T, Order> > &A, int numRows, int numCols)
{
    A.resize(numRows, numCols);
    random<dist>(A);
}

//------------------------------------------------------------------------------

template <Distribution dist>
struct random_
{
    random_(int numRows, int numCols)
        : _numRows(numRows), _numCols(numCols)
    {
    }

    template <typename I>
    void
    operator()(Matrix<I> &A) const
    {
        if (_numCols>0) {
            random<dist>(A.impl(), _numRows, _numCols);
        } else {
            random<dist>(A.impl());
        }
    }

    template <typename I>
    void
    operator()(Vector<I> &x) const
    {
        if (_numRows>0) {
            random<dist>(x.impl(), _numRows);
        } else {
            random<dist>(x.impl());
        }
    }

    int _numRows, _numCols;
};

//------------------------------------------------------------------------------

template <Distribution dist>
ResultClosure<random_<dist> >
random(int rows, int cols)
{
    return random_<dist>(rows, cols);
}

} // namespace flens












