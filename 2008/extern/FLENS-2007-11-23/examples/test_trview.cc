#include <flens/flens.h>

using namespace flens;
using namespace std;

template <typename FA>
void
test1(const GeMatrix<FA> &A)
{
    typedef typename GeMatrix<FA>::ConstTriangularView ConstTrView;
    typedef typename GeMatrix<FA>::ConstSymmetricView  ConstSyView;

    cout << "-- test 1 -------------------------------------" << endl;

    cout << "A = " << A << endl;

    ConstTrView U = upper(A);
    ConstTrView L = lowerUnit(A);

    cout << "U = " << U << endl;
    cout << "L = " << L << endl;

    ConstSyView SU = upper(A);
    ConstSyView SL = lower(A);

    cout << "SU = " << SU << endl;
    cout << "SL = " << SL << endl;
}

template <typename FA>
void
test2(GeMatrix<FA> &A)
{
    typedef typename GeMatrix<FA>::TriangularView TrView;
    typedef typename GeMatrix<FA>::SymmetricView  SyView;

    cout << "-- test 2 -------------------------------------" << endl;

    cout << "A = " << A << endl;

    TrView U = upper(A);
    TrView L = lowerUnit(A);

    cout << "U = " << U << endl;
    cout << "L = " << L << endl;

    SyView SU = upper(A);
    SyView SL = lower(A);

    cout << "SU = " << SU << endl;
    cout << "SL = " << SL << endl;
}

template <typename FA>
void
test3(const GbMatrix<FA> &A)
{
    typedef typename GbMatrix<FA>::ConstTriangularView ConstTrView;
    typedef typename GbMatrix<FA>::ConstSymmetricView  ConstSyView;

    cout << "-- test 3 -------------------------------------" << endl;

    cout << "A = " << A << endl;

    ConstTrView U = upper(A);
    ConstTrView L = lowerUnit(A);

    cout << "U = " << U << endl;
    cout << "L = " << L << endl;

    ConstSyView SU = upper(A);
    ConstSyView SL = lower(A);

    cout << "SU = " << SU << endl;
    cout << "SL = " << SL << endl;
}

template <typename FA>
void
test4(GbMatrix<FA> &A)
{
    typedef typename GbMatrix<FA>::TriangularView TrView;
    typedef typename GbMatrix<FA>::SymmetricView  SyView;

    cout << "-- test 4 -------------------------------------" << endl;

    cout << "A = " << A << endl;

    TrView U = upper(A);
    TrView L = lowerUnit(A);

    cout << "U = " << U << endl;
    cout << "L = " << L << endl;

    SyView SU = upper(A);
    SyView SL = lower(A);

    cout << "SU = " << SU << endl;
    cout << "SL = " << SL << endl;
}


int
main()
{
    GeMatrix<FullStorage<double, ColMajor> > A(5,5);
    A = 1, 2, 3, 4, 5,
        6, 7, 8, 9, 1,
        2, 3, 4, 5, 6,
        7, 8, 9, 1, 2,
        3, 4, 5, 6, 7;

    GbMatrix<BandStorage<double, ColMajor> > Ab(5,5,2,3);

    Ab.diag(-2) = 1, 3, 7;
    Ab.diag(-1) = 6, 7, 8, 9;
    Ab.diag( 0) = 1, 2, 3, 4, 5;
    Ab.diag( 1) = 6, 7, 8, 9;
    Ab.diag( 2) = 1, 3, 7;
    Ab.diag( 3) = 1, 8;

    test1(A);
    test2(A);
    test3(Ab);
    test4(Ab);
}












