#include <flens/flens.h>

using namespace flens;
using namespace std;

template <typename E>
void
dummy(const DenseVector<E> &x)
{
}

int
main()
{
    DenseVector<Array<double> > x(5), y;
    x = 1, 2, 3, 4, 5;

    GeMatrix<FullStorage<double, RowMajor> > A(5,5);
    A = 1, 2, 3, 4, 5,
        6, 7, 8, 9, 0,
        9, 7, 5, 3, 1,
        0, 2, 4, 6, 8,
        9, 1, 8, 2, 7;

    cout << "A = " << A << endl;
    cout << "x = " << x << endl;



    copy(A*x, y);
    cout << " -> 1) y = A*x = " << y << endl;



    y = A*x;
    cout << " 1) y = A*x = " << y << endl;

    y = x*A;
    cout << " 2) y = x'*A = " << y << endl;

    y = x*transpose(A);
    cout << " 3) y = x'*A' = " << y << endl;

    y = x*(1.5*A);
    cout << " 4) y = x'*A' = " << y << endl;

    y = x*(1.5*transpose(A));
    cout << " 5) y = x'*1.5*A' = " << y << endl;

    y = (1.5*x)*A;
    cout << " 6) y = 1.5*x'*A = " << y << endl;

    y = (1.5*x)*transpose(A);
    cout << " 7) y = 1.5*x'*A' = " << y << endl;

    y = x*(1.5*transpose(A));
    cout << " 8) y = x'*1.5*A' = " << y << endl;

    y = (1.5*x)*A;
    cout << " 9) y = 1.5*x'*A = " << y << endl;

    y = (1.5*x)*(1.5*transpose(A));
    cout << "10) y = 1.5*x'*1.5*A' = " << y << endl;

    y = 1.5*transpose(A)*x;
    cout << "11) y = 1.5*A'*x = " << y << endl;

    y  = transpose(1.5*A)*x;
    cout << "12) y = 1.5*A'*x = " << y << endl;

    y = 1.5*A*x;
    cout << "13) y = 1.5*A*x = " << y << endl;

    y = 1.5*(A*x);
    cout << "14) y = 1.5*(A*x) = " << y << endl;

    y = 2*x;
    cout << "15) y = 2*x" << y << endl;

    y = A*x + x;
    cout << "16) y = A*x + x" << y << endl;

    y += A*x + x;
    cout << "17) y += A*x + x" << y << endl;

    TrMatrix<FullStorage<double, RowMajor> > U(A.engine(), Upper);
    cout << "U = " << U << endl;

    x = U*x;
    cout << "17) x = U*x" << x << endl;

    x = x*U;
    cout << "18) x = x'*U = " << x << endl;

    x = 0.5*(U*x);
    cout << "17) x = 0.5*(U*x)" << x << endl;
/*
    y = x*(1.5*transpose(U));
    cout << "19) y = x'*1.5*U' = " << y << endl;

    y = (1.5*x)*U;
    cout << "20) y = 1.5*x'*U = " << y << endl;

    y = (1.5*x)*(1.5*transpose(U));
    cout << "21) y = 1.5*x'*1.5*U' = " << y << endl;

    y = 1.5*transpose(U)*x;
    cout << "22) y = 1.5*U'*x = " << y << endl;

    y  = transpose(1.5*U)*x;
    cout << "23) y = 1.5*U'*x = " << y << endl;

    y = 1.5*U*x;
    cout << "24) y = 1.5*U*x = " << y << endl;

    y = 1.5*(U*x);
    cout << "25) y = 1.5*(U*x) = " << y << endl;
    y = U*x + x;
    cout << "26) y = U*x + x" << y << endl;

    y += U*x + x;
    cout << "27) y += U*x + x" << y << endl;
*/

    DenseVector<Array<double> > z;
    z = 1.5*x;
    cout << "28) z = " << z << endl;

}












