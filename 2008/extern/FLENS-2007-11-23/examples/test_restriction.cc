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
    Restriction R;
    DenseVector<Array<double> > x(_(0,8)),
                                y(_(0,4));

    x = 0, 1, 3, 3, 5, 5, 7, 7, 0;
    y = R*x;

    cout << "x = " << x << endl;
    cout << "y = " << y << endl;

}












