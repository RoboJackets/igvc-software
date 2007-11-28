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

Restriction  R;
Prolongation P;

template <typename M, typename V>
void
smooth(const M &S, V &u)
{
    for (int k=1; k<=10; ++k) {
        u = S*u;
    }
}


void
test(double omega, int nr)
{
    //-- fine grid -------------------------------------------------------------
    int    N = 63;
    double h = 1./(N+1);

    Poisson1D   A(N+1);
    DDeVector  u(_(0,N+1)), f(_(0,N+1)), r(_(0,N+1));

    //-- smooth on fine grid ---------------------------------------------------
    SnapShot snap("res_relax");

    for (int k=1; k<=N; ++k) {
        double x = k*h;
        u(k) = sin(27*M_PI*x) + sin(3*M_PI*x);
    }
    smooth(GaussSeidel<Poisson1D, DDeVector>(A, f, omega), u);
    r = f - A*u;
    snap(nr) << r << endl;


}

int
main()
{
    test(0.7, 1);
    test(0.8, 2);
    test(0.9, 3);
    test(1.0, 4);
    test(1.1, 5);
    test(1.2, 6);
    test(1.3, 7);
    test(1.4, 8);
    test(1.5, 9);
    test(1.6, 10);
    test(1.7, 11);
    test(1.8, 12);
    test(1.9, 13);
}












