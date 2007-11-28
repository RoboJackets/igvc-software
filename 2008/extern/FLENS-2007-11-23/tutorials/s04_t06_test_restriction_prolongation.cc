#include <flens/flens.h>

//-- here our flens extensions ---------------
#include <tutorials/gridvector2d.h>
#include <tutorials/gridvector2d_blas.h>
#include <tutorials/restriction.h>
#include <tutorials/prolongation.h>
//--------------------------------------------

using namespace flens;
using namespace std;

int
main()
{
    int n = 7;
    int N = 3;
    GridVector2D x(n+1);

    x.grid = 0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 1, 1, 1, 1, 1, 1, 1, 0,
             0, 1, 2, 2, 2, 2, 2, 1, 0,
             0, 1, 2, 3, 3, 3, 2, 1, 0,
             0, 1, 2, 3, 4, 3, 2, 1, 0,
             0, 1, 2, 3, 3, 3, 2, 1, 0,
             0, 1, 2, 2, 2, 2, 2, 1, 0,
             0, 1, 1, 1, 1, 1, 1, 1, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0;
             
    GridVector2D x_c(N+1);
    Restriction  R;
    x_c = R*x;

    cout << "x = " << x.grid(_(1,n),_(1,n)) << endl;
    cout << "x_c = " << x_c.grid(_(1,N),_(1,N)) << endl;

    GridVector2D x_p(n+1);
    Prolongation  P;
    x_p += P*x_c;
    cout << "x_p = " << x_p.grid(_(1,n),_(1,n)) << endl;
}
