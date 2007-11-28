#include <flens/flens.h>

using namespace flens;
using namespace std;

int
main()
{
    GeMatrix<FullStorage<double, ColMajor> > A(3,3);
    A = 1, 2, 3,
        4, 5, 6,
        7, 8, 9;
    
    DenseVector<Array<double> > x(3), y(3);
    x = 1, 2, 3;

    y = A*x;
    cout << "y = " << y << endl;

    x = A*x;
    cout << "x = " << x << endl;
}