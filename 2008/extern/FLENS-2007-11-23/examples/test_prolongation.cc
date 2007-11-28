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
    Prolongation P;
    DenseVector<Array<double> > x(_(0,4)),
                                y(_(0,8));

    x = 0, 1, 2, 3, 0;
    y += P*x;

    cout << "x = " << x << endl;
    cout << "y = " << y << endl;

}












