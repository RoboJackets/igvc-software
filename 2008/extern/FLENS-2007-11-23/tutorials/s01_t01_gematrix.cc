#include <flens/flens.h>


using namespace flens;
using namespace std;

typedef GeMatrix<FullStorage<double, ColMajor> >  GEMatrix;

int
main()
{
    GEMatrix A(4,4);
    A = 1, 2, 3, 4,
        5, 6, 7, 8,
        9, 8, 7, 6,
        5, 4, 3, 20;

    cout << "A = " << A << endl;

    cout << "Dim. of A: " << A.numRows() << " x " << A.numCols() << endl;
    cout << endl;

    cout << "Row indices: " << A.firstRow() << ".." << A.lastRow() << endl;
    cout << endl;

    cout << "Col indices: " << A.firstCol() << ".." << A.lastCol() << endl;
    cout << endl;

    // element access (write)
    A(3,2) = 42;

    // element access (read)
    cout << "changed element: A(3,2) = " << A(3,2) << endl;
    cout << endl;

    cout << "A = " << A << endl;

    return 0;
}
