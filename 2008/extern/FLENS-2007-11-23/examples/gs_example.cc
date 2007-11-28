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
    int N = 32;
    //int    N = 63;
    double h = 1./(N+1);


    Poisson1D                         A(N+1);
    DenseVector<Array<double> >       u(_(0,N+1)), f(_(0,N+1)), r(_(0,N+1));
    GaussSeidel<Poisson1D, DDeVector> S(A, f, 1.);


    for (int k=1; k<=N; ++k) {
        double x = k*h;
        u(k) = sin(27*M_PI*x) + sin(3*M_PI*x);
    }

    SnapShot initial("gs_initial");
    initial(0) << u << endl;

    SnapShot snap("gs_example");
    for (int k=1; k<=30; ++k) {
        r = f - A*u;
        snap(k) << r << endl;

        u = S*u;
    }
}












