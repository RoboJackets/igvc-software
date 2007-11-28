#include <tests/evaltest.h>

void
test1()
{
    GbMatrix<BandStorage<double, ColMajor> > A(3,4,1,2);
    
    A.diag(-1) = 1, 2;
    A.diag( 0) = 3, 4, 5;
    A.diag( 1) = 6, 7, 8;
    A.diag( 2) = 9, -1;
    
    cerr << "A = " << A << endl;
    
    A *= 3;

    cerr << "A = " << A << endl;
}

void
test2()
{
    GbMatrix<BandStorage<double, ColMajor> > A(3,4,1,2), B(3,4,1,2), C;
    
    A.diag(-1) = 1, 2;
    A.diag( 0) = 3, 4, 5;
    A.diag( 1) = 6, 7, 8;
    A.diag( 2) = 9, -1;

    B.diag(-1) = 1, 2;
    B.diag( 0) = 3, 4, 5;
    B.diag( 1) = 6, 7, 8;
    B.diag( 2) = 9, -1;
    
    B *=-1;
    
    cerr << "A = " << A << endl;
    cerr << "B = " << B << endl;
    
    C = A + B;
    
    A += B;

    cerr << "A = " << A << endl;
    cerr << "C = " << C << endl;

}

void
test3()
{
    GbMatrix<BandStorage<double, ColMajor> > A(3,4,1,2), B(3,4,1,2);
    
    A.diag(-1) = 1, 2;
    A.diag( 0) = 3, 4, 5;
    A.diag( 1) = 6, 7, 8;
    A.diag( 2) = 9, -1;

    B.diag(-1) = 1, 2;
    B.diag( 0) = 3, 4, 5;
    B.diag( 1) = 6, 7, 8;
    B.diag( 2) = 9, -1;
    
    A = A - B;

    cerr << "A = " << A << endl;
}

void
test4()
{
    GbMatrix<BandStorage<double, ColMajor> > A(3,4,1,2), B(3,4,1,2);
    
    A.diag(-1) = 1, 2;
    A.diag( 0) = 3, 4, 5;
    A.diag( 1) = 6, 7, 8;
    A.diag( 2) = 9, -1;

    B.diag(-1) = 1, 2;
    B.diag( 0) = 3, 4, 5;
    B.diag( 1) = 6, 7, 8;
    B.diag( 2) = 9, -1;
    
    A = B - A;

    cerr << "A = " << A << endl;
}


int
main()
{
    run("test1", test1, true);
    run("test2", test2, true);
    run("test3", test3, true);
    run("test4", test4, false);
}

