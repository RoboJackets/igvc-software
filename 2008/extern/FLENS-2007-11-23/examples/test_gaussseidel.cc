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

typedef DenseVector<Array<double> > DDeVector;

int
main()
{
    Poisson1D                         A(8);
    DDeVector                         x(_(0,8)), f(_(0,8));
    GaussSeidel<Poisson1D,DDeVector>  S(A, f, 1.3);

    // fuer verschieden x testen

    f = 1, 1, 1, 1, 1, 1, 1;

    SnapShot snap("test_gs");
    for (int k=1; k<=50; ++k) {
        x = S*x;
        snap(k) << x;
    }
}












