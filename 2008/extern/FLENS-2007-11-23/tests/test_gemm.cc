#include <tests/evaltest.h>

void
test1()
{
    GeMatrix<FullStorage<double, ColMajor> > A(3,3), B(3,3), C;

    A = 1, 2, 3,
        4, 5, 6,
        7, 8, 9;
        
    B = 9, 8, 7,
        6, 5, 4,
        3, 2, 1;
    
    cerr << "A = " << A << endl;
    cerr << "B = " << B << endl;
    
    C = A*B;
    cerr << "C = A*B = " << C << endl;

    C = (1.5*A)*B;
    cerr << "C = (1.5*A)*B = " << C << endl;

    C = A*(1.5*B);
    cerr << "C = A*(1.5*B) = " << C << endl;

    C = (3*A)*(0.5*B);
    cerr << "C = (3*A)*(0.5*B) = " << C << endl;

    C = transpose(A)*B;
    cerr << "C = transpose(A)*B = " << C << endl;

    C = A*transpose(B);
    cerr << "C = A*transpose(B) = " << C << endl;

    C = transpose(A)*transpose(B);
    cerr << "C = transpose(A)*transpose(B) = " << C << endl;
    
    C = transpose(A)*(3*B);
    cerr << "C = transpose(A)*(3*B) = " << C << endl;

    C = (3*A)*transpose(B);
    cerr << "C = (3*A)*transpose(B) = " << C << endl;
}

int
main()
{
    run("test1", test1, true);
}

