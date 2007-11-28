#include <flens/flens.h>

using namespace flens;
using namespace std;

typedef GeMatrix<FullStorage<double, ColMajor> >   GEMatrix;
typedef DenseVector<Array<double> >                DEVector;

int
main()
{
    GEMatrix A(4,4);
    A = 1, 2, 3, 4,
        5, 6, 7, 8,
        9, 8, 7, 6,
        5, 4, 3, 2;
        
    DEVector x(4), y(4), z(4);
    x = 1, 2, 3, 4;
    y = 5, 6, 7, 8;
    
    // compute: z = 2*A'*x
    gemv(ColMajor, Trans,
         A.numRows(), A.numCols(), 2., A.data(), A.leadingDimension(),
         x.data(), x.stride(),
         0., z.data(), z.stride());
         
    // compute: z += 1.5*y
    axpy(z.length(), 1.5, y.data(), y.stride(), z.data(), z.stride());
    
    cout << "A = " << A << endl;
    cout << "x = " << x << endl;
    cout << "y = " << y << endl;
    cout << "z = 2*A'*x + 1.5*y = " << z << endl;
}
