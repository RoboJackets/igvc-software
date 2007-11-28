#include <flens/flens.h>

using namespace flens;
using namespace std;

template <typename SA, typename VA>
void
init(SparseGeMatrix<SA> &A, DenseVector<VA> &x, DenseVector<VA> &b)
{
    int n= A.numRows();
    for (int i=1; i<=n; ++i) {
        if (i>1) {
            A(i, i-1) = -1;
        }
        A(i,i) = 2;
        if (i<n) {
            A(i, i+1) = -1;
        }
        b(i) = i % 4;
        x(i) = i*(i+1) % 5;
    }    

    A.finalize();
}

int
main()
{
    int n = 10;
    SparseGeMatrix<CRS<double> > A(n, n);
    DenseVector<Array<double> >  x(n), b(n), y(n);

    init(A, x, b);

    cout << "x = " << x << endl;
    cout << "b = " << b << endl;

    y = 1.5*transpose(A)*x + b;
    cout << "y = 1.5A'x + b = " << y << endl;


    y = b + 1.5*A*x;
    cout << "y = b + 1.5Ax = " << y << endl;
}
