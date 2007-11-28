#include <flens/flens.h>

using namespace flens;
using namespace std;


int
main()
{
    SbMatrix<BandStorage<double, ColMajor> > A(5, Upper, 1);
    A.diag(0) =  2,  3,  4,  3, 2;
    A.diag(1) = -1, -2, -2, -1;
    
    A /= 6*6;

    DenseVector<Array<double> > b(5), x(5);
    b = 1;

    int it = cg(A, x, b);
    cout << "A = " << A << endl;
    cout << "b = " << b << endl;
    cout << "number of iterations: " << it << endl;
    cout << "x = " << x << endl;
}
