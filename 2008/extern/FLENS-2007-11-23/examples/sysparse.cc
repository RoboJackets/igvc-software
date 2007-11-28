#include <flens/flens.h>

using namespace flens;
using namespace std;

/*
 *  Sparse matrices are used in two SEPARATE phases
 *  1) initialization
 *  2) using the matrices (e.g. solving a system of linear equations)
 *
 *  Once the initialization is completed vales of the matrix can not be
 *  changed.
 */

int
main()
{
    SparseSymmetricMatrix<CRS<double> > A(5,5);

    /* Note:
     *  A(i,j) reference the same element as A(j,i) as A is symmetric
     * that means
     *    A(1,3) = 4;
     *    A(3,1) = 2;
     * results in A(1,3) == A(3,1) == 2
     *
     * new matrix elements are initialized with zero, i.e you safely can
     * use without pervious initialization "+=":
     *    A(1,2) += 2;
     *    A(2,1) += 1;
     * results in A(1,2) == 3
     */

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

    A(5,2) +=  8;
    A(5,5) += -5;

    // complete initialization
    A.finalize();

    // print internal data structure
    cout << "A.values = " << A.engine().values << endl;
    cout << "A.columns = " << A.engine().columns << endl;
    cout << "A.rows = " << A.engine().rows << endl;

    // print all allocated elements.
    // Note:
    //   the symmetric matrix stores only elements on the upper half
    cout << "A = " << A << endl;

    return 0;
}













