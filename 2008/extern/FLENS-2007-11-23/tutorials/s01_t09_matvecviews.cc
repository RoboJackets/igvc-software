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

    GEMatrix::TriangularView U = upper(A);
    GEMatrix::TriangularView L = lowerUnit(A);
              
    GEMatrix::SymmetricView SU = upper(A);
    GEMatrix::SymmetricView SL = lower(A);
    
    cout << "U =  " << U << endl;
    cout << "L =  " << L << endl;
    cout << "SU = " << SU << endl;
    cout << "SL = " << SL << endl;
                
    return 0;
}
