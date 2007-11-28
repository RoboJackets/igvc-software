#include <flens/flens.h>
#include <tutorials/poisson1d.h>

using namespace flens;
using namespace std;

int
main()
{
    int n=5;
    Poisson1D A(n+1);

    DenseVector<Array<double> > b(_(0,n+1)), x(_(0,n+1));
    b(_(1,n)) = 1;

    int it = cg(A, x, b);

    cout << "it = " << it << endl;
    cout << "x = " << x << endl;
}
