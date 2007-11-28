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
    DenseVector<Array<double> >  x(n), b(n), r(n);

    for (int i=1; i<=n; ++i) {
        x(i) = 1;
    }
    for (int i=1; i<=k*n; ++i) {
        int col = rand() % n +1;
        int row = rand() % n +1;
        int incr = rand() % n +1;

        A(row, col) += incr;
        
        row = rand() % n +1;
        incr = rand() % n +1;
        x(row) += incr;

        row = rand() % n +1;
        incr = rand() % n +1;
        b(row) += incr;

    }

    std::cerr << "initialization: " << t.toc() << "s" << std::endl;

    t.tic();
    A.finalize();
    std::cerr << "finalization:   " << t.toc() << "s" << std::endl;

    t.tic();
    for (int numIt=0; numIt<10; ++numIt) {
        r = b - A*x;
    }
    std::cerr << "computations:   " << t.toc() << "s" << std::endl;
}
