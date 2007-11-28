#include <flens/flens.h>

using namespace flens;
using namespace std;

int
main()
{
    int n = 8;
    SparseGeMatrix<CRS<double> > A(n, n);

    // initialize matrix
    for (int i=1; i<=n; ++i) {
        if (i>1) {
            A(i, i-1) = -1;
        }
        A(i,i) = 2;
        if (i<n) {
            A(i, i+1) = -1;
        }
    }

    // end initialization phase 
    A.finalize();

    // use A for computations
    DenseVector<Array<double> >  x(n), b(n), r(n);
    for (int i=1; i<=n; ++i) {
        x(i) = i;
        b(i) = n-1;
    }

    r = b - A*x;

    // output A and residual r
    cout << "A = " << A << endl;
    cout << "r = " << r << endl;
}
