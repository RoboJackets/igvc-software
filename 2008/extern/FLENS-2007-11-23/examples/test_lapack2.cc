#include <flens/flens.h>

using namespace flens;
using namespace std;

void
test1()
{
    DenseVector<Array<int> > P(5);
    GbMatrix<BandStorage<double, ColMajor> > A(5,5,1,2);

    A.diag(1) = -1, -1, -1, -1;
    A.diag(0) =  2,  2,  2,  2, 2;
    A.diag(-1) = -1, -1, -1, -1;


    cout << "A = " << A << endl;
    trf(A, P);

    cout << "lu(A) = " << A << endl;

    GeMatrix<FullStorage<double, ColMajor> > B(5,2);
    B = 1, 2,
        3, 4,
        5, 6,
        7, 8,
        9, 1;

    cout << "B = " << B << endl;

    trs(NoTrans, A, P, B);

    cout << "X = " << B << endl;
}

void
test2()
{
    DenseVector<Array<int> > P(5);
    GbMatrix<BandStorage<double, ColMajor> > A(5,5,1,2);

    A.diag(1) = -1, -1, -1, -1;
    A.diag(0) =  2,  2,  2,  2, 2;
    A.diag(-1) = -1, -1, -1, -1;

    GeMatrix<FullStorage<double, ColMajor> > B(5,2);
    B = 1, 2,
        3, 4,
        5, 6,
        7, 8,
        9, 1;

    cout << "A = " << A << endl;
    cout << "B = " << B << endl;

    sv(A, P, B);
    cout << "lu(A) = " << A << endl;
    cout << "X = " << B << endl;
}

int
main()
{
    test1();
    test2();
    return 0;
}












