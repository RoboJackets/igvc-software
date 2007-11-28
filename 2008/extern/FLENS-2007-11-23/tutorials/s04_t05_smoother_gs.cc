#include <flens/flens.h>

//-- here our flens extensions ---------------
#include <tutorials/poisson2d.h>
#include <tutorials/gridvector2d.h>
#include <tutorials/gridvector2d_blas.h>
#include <tutorials/smoother_gs.h>
//--------------------------------------------

//-- would be easier with sprintf ;-) --------
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
//--------------------------------------------

using namespace flens;
using namespace std;

void
initialGuess(GridVector2D::Grid &X)
{
    int m = X.lastRow()-1,
        n = X.lastCol()-1;

    double hx = 1./(n+1),
           hy = 1./(m+1);
    
    int f1 = 3,  f2 = 40,
        f3 = 20, f4 = 60,
        f5 = 4,  f6 = 6;

    for (int i=1; i<=m; ++i) {
        for (int j=1; j<=n; ++j) {
            double x=hx*i;
            double y=hy*j;
            X(i,j) = sin(M_PI*f1*x)*sin(M_PI*f2*y)
                   + sin(M_PI*f3*x)*sin(M_PI*f4*y)
                   + sin(M_PI*f5*x)*sin(M_PI*f6*y);
        }
    }
}

int
main()
{
    int n = 128;
    Poisson2D                           A(n+1);
    GridVector2D                        b(n+1), x(n+1);
    SmootherGS<Poisson2D, GridVector2D> S(A, b);

    initialGuess(x.grid);

    for (int k=1; k<=31; ++k) {
        x = S*x;
        
        // save x in snapshot file
        std::ostringstream s;
        s << "smoothing_" << std::setw(3) << std::setfill('0') << k << ".dat";
        ofstream file(s.str().c_str());
        file << x << endl;
    }
}

