#include <flens/flens.h>
#include <tutorials/timer.h>

using namespace flens;
using namespace std;

int
main()
{
    timer t;

    int n = 1000000;
    int k = 5;

    t.tic();

    SparseGeMatrix<CRS<double> > A(n, n, k);
    DenseVector<Array<double> >  x(n), y(n);

    for (int i=1; i<=n; ++i) {
        x(i) = 1;
    }
    for (int i=1; i<=k*n; ++i) {
        int col = rand() % n +1;
        int row = rand() % n +1;
        int incr = rand() % n +1;

        A(row, col) += incr;
    }

    std::cerr << "initialization: " << t.toc() << "s" << std::endl;

    t.tic();
    A.finalize();
    std::cerr << "finalization:   " << t.toc() << "s" << std::endl;

    t.tic();
    for (int numIt=0; numIt<10; ++numIt) {
        y = A*x;
    }
    std::cerr << "y = A*x:        " << t.toc() << "s" << std::endl;

    t.tic();
    for (int numIt=0; numIt<10; ++numIt) {
        y = transpose(A)*x;
    }
    std::cerr << "y = A'*x:       " << t.toc() << "s" << std::endl;
}
