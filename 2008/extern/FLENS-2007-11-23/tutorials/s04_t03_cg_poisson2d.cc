#include <flens/flens.h>

//-- here our flens extensions ---------
#include <tutorials/poisson2d.h>
#include <tutorials/gridvector2d.h>
#include <tutorials/gridvector2d_blas.h>
//--------------------------------------

using namespace flens;
using namespace std;

int
main()
{
    int n = 128;
    Poisson2D     A(n+1);
    GridVector2D  b(n+1), x(n+1);
    b.grid(_(1,n),_(1,n)) = 1;

    int it = cg(A, x, b);
    cout << "it = " << it << endl;

    ofstream file("poisson2d.dat");

    file << x << endl;
}
