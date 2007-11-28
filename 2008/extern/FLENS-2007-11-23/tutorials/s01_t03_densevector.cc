#include <flens/flens.h>

using namespace flens;
using namespace std;

typedef DenseVector<Array<double> >  DEVector;

int
main()
{
    DEVector x(4), y(_(0,3)), z1, z2;
    x = 1, 2, 3, 4;
    y = 5, 6, 7, 8;
    
    cout << "x = " << x << endl;
    cout << "y = " << y << endl;
    cout << endl;

    cout << "Some info about y:" << endl;
    cout << "  length of y =      " << y.length() << endl;
    cout << "  first index of y = " << y.firstIndex() << endl;
    cout << "  last index of y =  " << y.lastIndex() << endl;
    cout << endl;
    
    z1 = x;
    cout << "z1 = x = " << z1 << endl;
    z1 = y;
    cout << "z1 = y = " << z1 << endl;
    cout << endl;

    z2 = y;
    cout << "z2 = y = " << z2 << endl;
    z2 = x;
    cout << "z2 = x = " << z2 << endl;
    
    
    // element access (write)
    x(2) = -2;
    
    // element access (read)
    cout << "x(2) = " << x(2) << endl;
}
