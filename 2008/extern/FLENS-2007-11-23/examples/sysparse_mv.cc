#include <flens/flens.h>

using namespace flens;
using namespace std;


//== Actual implementation of matrix vector product ============================

template <typename T, typename VX, typename VY>
void
sysp_mv(const DenseVector<Array<T> > &values,
        const DenseVector<Array<int> > &columns,
        const DenseVector<Array<int> > &rows,
        const DenseVector<VX> &x,
        DenseVector<VY> &y)
{
    // init y with zeros
    for (int i=1; i<=y.length(); ++i) {
        y(i) = 0;
    }

    int i = 0;
    for (int k=1; k<=values.length(); ++k) {
        if (k>=rows(i+1)) {
            ++i;
        }
        int j = columns(k);

        // now values(k) represents A(i,j)
        y(i) += values(k)*x(j);

        if (i==j) {
            continue;
        }

        // and A(i,j) equals A(j,i)
        y(j) += values(k)*x(i);
    }
}

//==============================================================================

template <typename T, typename VX, typename VY>
void
mv(const SparseSymmetricMatrix<CRS<T> > &A,
   const DenseVector<VX> &x,
   DenseVector<VY> &y)
{
    assert(A.numCols()==x.length());

    if (y.length()!=A.numRows()) {
        y.resize(A.numRows());
    }
    sysp_mv(A.engine().values, A.engine().columns, A.engine().rows, x, y);
}

int
main()
{
    SparseSymmetricMatrix<CRS<double> > A(5,5);

    A(1,1) +=  1;
    A(1,2) += -1;
    A(1,4) += -3;

    A(2,1) += -2;
    A(2,2) +=  5;

    A(3,3) +=  4;
    A(3,4) +=  6;
    A(3,5) +=  4;

    A(4,1) += -4;
    A(4,3) +=  2;
    A(4,4) +=  7;

    A(5,2) =  8;
    A(5,5) = -5;
    A.finalize();

    cout << "A = " << A << endl;

    DenseVector<Array<double> > x(5), y(5);
    x = 1, 2, 3, 4, 5;

    mv(A,x,y);

    cout << "y = " << y << endl;
    return 0;
}













