#include <flens/flens.h>

using namespace flens;
using namespace std;

void
test1()
{
    int lapackInfo;
    DenseVector<Array<int> > P(5);
    GeMatrix<FullStorage<double, ColMajor> > A(5,5);

    A = 1,     2,     3,     4,    79,
        6,     1,     5,     2,     4,
        5,    29,     4,     3,     1,
        1,     3,     5,     7,     9,
        5,     7,     9,    11,    13;

    cout << "A = " << A << endl;
    lapackInfo = trf(A, P);

    cout << "lu(A) = " << A << endl;
    cout << "P     = " << P << endl;
    cout << "lapackInfo = " << lapackInfo << endl << endl;

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
    int lapackInfo;
    DenseVector<Array<int> > P(5);
    GeMatrix<FullStorage<double, ColMajor> > A(5,5);

    A = 1,     2,     3,     4,    79,
        6,     1,     5,     2,     4,
        5,    29,     4,     3,     1,
        1,     3,     5,     7,     9,
        5,     7,     9,    11,    13;

    GeMatrix<FullStorage<double, ColMajor> > B(5,2);
    B = 1, 2,
        3, 4,
        5, 6,
        7, 8,
        9, 1;

    cout << "A = " << A << endl;
    cout << "B = " << B << endl;

    lapackInfo = sv(A, P, B);
    cout << "lapackInfo = " << lapackInfo << endl << endl;

    cout << "X = " << B << endl;
}

template <typename MA>
TrMatrix<MA>
lowerUnitTr(GeMatrix<MA> &A)
{
    return TrMatrix<MA>(A.engine(), Lower, Unit);
}

template <typename MA>
TrMatrix<MA>
lowerUnitTr(const GeMatrix<MA> &A)
{
    return TrMatrix<MA>(A.engine(), Lower, Unit);
}

template <typename MA>
TrMatrix<MA>
upperTr(GeMatrix<MA> &A)
{
    return TrMatrix<MA>(A.engine(), Upper);
}

template <typename MA>
TrMatrix<MA>
upperTr(const GeMatrix<MA> &A)
{
    return TrMatrix<MA>(A.engine(), Upper);
}

void
test3()
{
    int lapackInfo;
    DenseVector<Array<int> > P(5);
    GeMatrix<FullStorage<double, ColMajor> > A(5,5);

    A = 2,    -1,     0,     0,     0,
       -1,     2,    -1,     0,     0,
        0,    -1,     2,    -1,     0,
        0,     0,    -1,     2,    -1,
        0,     0,     0,    -1,     2;

    cout << "A = " << A << endl;
    lapackInfo = trf(A, P);

    cout << "lu(A) = " << A << endl;
    cout << "P = " << P << endl;
    cout << "lapackInfo = " << lapackInfo << endl << endl;

    GeMatrix<FullStorage<double, ColMajor> > B(5,2);
    B = 1, 2,
        3, 4,
        5, 6,
        7, 8,
        9, 1;

    cout << "B = " << B << endl;

    trs(NoTrans, lowerUnitTr(A), B);
    trs(NoTrans, upperTr(A), B);

    cout << "X = " << B << endl;
}

void
test4()
{
    int lapackInfo;
    DenseVector<Array<double> > tau(5);
    GeMatrix<FullStorage<double, ColMajor> > A(5,5);

    A = 2,    -1,     0,     0,     0,
       -1,     2,    -1,     0,     0,
        0,    -1,     2,    -1,     0,
        0,     0,    -1,     2,    -1,
        0,     0,     0,    -1,     2;

    cout << "A = " << A << endl;
    lapackInfo = qrf(A, tau);

    cout << "qr(A) = " << A << endl;
    cout << "tau = " << tau << endl;
    cout << "lapackInfo = " << lapackInfo << endl << endl;

    GeMatrix<FullStorage<double, ColMajor> > B(5,2);
    B = 1, 2,
        3, 4,
        5, 6,
        7, 8,
        9, 1;

    cout << "B = " << B << endl;

    ormqr(Left, Trans, A, tau, B);

    cout << "Q'*B = " << B << endl;

    trs(NoTrans, upperTr(A), B);

    cout << "X = " << B << endl;
}

void
test5()
{
    int lapackInfo;
    DenseVector<Array<double> > tau(5);
    GeMatrix<FullStorage<double, ColMajor> > A(5,5);

    A = 2,    -1,     0,     0,     0,
       -1,     2,    -1,     0,     0,
        0,    -1,     2,    -1,     0,
        0,     0,    -1,     2,    -1,
        0,     0,     0,    -1,     2;

    GeMatrix<FullStorage<double, ColMajor> > B(5,2);
    B = 1, 2,
        3, 4,
        5, 6,
        7, 8,
        9, 1;

    cout << "A = " << A << endl;
    cout << "B = " << B << endl;

    lapackInfo = ls(NoTrans, A, B);
    cout << "lapackInfo = " << lapackInfo << endl << endl;

    cout << "X = " << B << endl;
}

void
test6()
{
    int lapackInfo;
    DenseVector<Array<int> > P(5);
    GeMatrix<FullStorage<double, ColMajor> > A(5,5);

    A = 1,     2,     3,     4,    79,
        6,     1,     5,     2,     4,
        5,    29,     4,     3,     1,
        1,     3,     5,     7,     9,
        5,     7,     9,    11,    13;

    cout << "A = " << A << endl;

    DenseVector<Array<double> >              wr(5), wi(5);
    GeMatrix<FullStorage<double, ColMajor> > vl(5,5), vr(5,5);

    lapackInfo = ev(true, true, A, wr, wi, vl, vr);
    cout << "lapackInfo = " << lapackInfo << endl;
    cout << "wr = " << wr << endl;
    cout << "wi = " << wi << endl;
    cout << "vl = " << vl << endl;
    cout << "vr = " << vr << endl;
}

void
test7()
{
    int lapackInfo;
    DenseVector<Array<int> > P(5);
    GeMatrix<FullStorage<complex<double>, ColMajor> > A(5,5);

    A = 1,     2,     3,     4,    79,
        6,     1,     5,     2,     4,
        5,    29,     4,     3,     1,
        1,     3,     5,     7,     9,
        5,     7,     9,    11,    13;

    cout << "A = " << A << endl;

    DenseVector<Array<complex<double> > >              w(5);
    GeMatrix<FullStorage<complex<double>, ColMajor> >  vl(5,5), vr(5,5);

    lapackInfo = ev(true, true, A, w, vl, vr);
    cout << "lapackInfo = " << lapackInfo << endl;
    cout << "w = " << w << endl;
    cout << "vl = " << vl << endl;
    cout << "vr = " << vr << endl;
}

int
main()
{
    test1();
    test2();
    test3();
    test4();
    test5();
    test6();
    test7();
}












