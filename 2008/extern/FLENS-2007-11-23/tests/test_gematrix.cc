#include <tests/evaltest.h>

void
test1()
{
    GeMatrix<FullStorage<double, ColMajor> > A(3,4);
    
    A =  3,  6,  9,  0,
         1,  4,  7, -1,
         0,  2,  5,  8;
    
    cerr << "A = " << A << endl;
    
    A *= 3;

    cerr << "A = " << A << endl;
}

void
test2()
{
    GeMatrix<FullStorage<double, ColMajor> > A(3,4), B(3,4), C;
    
    A =  3,  6,  9,  0,
         1,  4,  7, -1,
         0,  2,  5,  8;

    B =  3,  6,  9,  0,
         1,  4,  7, -1,
         0,  2,  5,  8;
    
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
    GeMatrix<FullStorage<double, ColMajor> > A(3,4), B(3,4);
    
    A =  3,  6,  9,  0,
         1,  4,  7, -1,
         0,  2,  5,  8;

    B =  3,  6,  9,  0,
         1,  4,  7, -1,
         0,  2,  5,  8;
    
    A = A - B;

    cerr << "A = " << A << endl;
}

void
test4()
{
    GeMatrix<FullStorage<double, ColMajor> > A(3,4), B(3,4);
    
    A =  3,  6,  9,  0,
         1,  4,  7, -1,
         0,  2,  5,  8;

    B =  3,  6,  9,  0,
         1,  4,  7, -1,
         0,  2,  5,  8;
    
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

