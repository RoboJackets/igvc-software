#include <flens/flens.h>
#include <iostream>

using namespace flens;
using namespace std;

typedef DenseVector<Array<double> >                DEVector;

int
main()
{
    DEVector x(6);
    x = 1, 2, 3, 4, 5, 6;
    
    // views
    DEVector::View y = x(_(2,6));
    DEVector::View z = y(_(1,2,5));
    
    // copy
    DEVector c = x(_(2,6));
    
    cout << "Vectors x,c and views y, z:" << endl;
    cout << " x = " << x << endl;
    cout << " c = " << c << endl;
    cout << " y = " << y << endl;
    cout << " z = " << z << endl;
    cout << endl;
    
    
    // modify views:
    y(2) = 123;
    z(3) = 456;
    cout << "y(2) = 123;" << endl;
    cout << "z(3) = 456;" << endl;
    cout << endl;

    cout << "Here again x, c, y and z:" << endl;
    cout << " x = " << x << endl;
    cout << " c = " << c << endl;
    cout << " y = " << y << endl;
    cout << " z = " << z << endl;
}
