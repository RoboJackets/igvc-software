#include <cassert>
#include <cmath>
#include <flens/flens.h>
#include <iostream>

#include <examples/poisson1d.h>
#include <examples/gaussseidel.h>
#include <examples/restriction.h>
#include <examples/prolongation.h>

using namespace flens;
using namespace std;

int
main()
{
    Poisson1D A(8);
    DenseVector<Array<double> > x(_(0,8)), y;

    // fuer verschieden x testen

    x = 0, 1, 1, 1, 1, 1, 1, 1, 0;
    //x = 0, 1, 2, 3, 4, 5, 6, 7, 0;
    //x = 0, 1, 2, 1, 2, 1, 2, 1, 0;

    y = A*x;

    cout << "y = A*x = " << y << endl;
}












