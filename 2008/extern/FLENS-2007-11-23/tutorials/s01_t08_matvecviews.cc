#include <flens/flens.h>
#include <iostream>

using namespace flens;
using namespace std;

typedef GeMatrix<FullStorage<double, ColMajor> >   GEMatrix;

int
main()
{
    GEMatrix A(5,5);
    A = 1, 2, 3, 4, 5,
        6, 7, 8, 9, 0,
        9, 7, 5, 3, 1,
        0, 9, 1, 8, 2,
        7, 3, 6, 4, 5;

    // B reference elements in rows 2 and 3
    GEMatrix::ConstView B = A(_(2,3),_);
    
    // x references all elements in the 5th column
    GEMatrix::ConstVectorView x = A(_,5);
    
    // y references in the 1st row, the first two elements
    GEMatrix::VectorView y = A(1,_(1,2));

    cout << "original matrix:" << endl;
    cout << "A = " << A << endl;

    cout << "views referencing parts of A:" << endl;
    cout << "B = " << B << endl;
    cout << "x = " << x << endl;
    cout << "y = " << y << endl;
    
    y = B*x;
    cout << "computing: y = B*x = " << y << endl;
    
    cout << "A = " << A << endl;
    
    return 0;
}
