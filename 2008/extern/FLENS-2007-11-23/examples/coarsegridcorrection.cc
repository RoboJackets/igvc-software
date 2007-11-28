#include <cassert>
#include <cmath>
#include <flens/flens.h>
#include <iostream>

#include <examples/poisson1d.h>
#include <examples/gaussseidel.h>
#include <examples/restriction.h>
#include <examples/prolongation.h>

namespace flens {

template <typename MA, typename VB, typename VX>
void
solve(const MA &A, const VB &b, VX &x)
{
    GaussSeidel<MA, VB> GS(A, b, 1.3);
    for (int k=0; k<=10; ++k) {
        x = GS*x;
    }
}

} // namespace flens

using namespace flens;
using namespace std;

typedef DenseVector<Array<double> > DDeVector;

Restriction  R;
Prolongation P;

int
main()
{
    //-- fine grid -------------------------------------------------------------
    int    N = 63;
    double h = 1./(N+1);

    Poisson1D                         A(N+1);
    DDeVector                         u(_(0,N+1)), f(_(0,N+1)), r(_(0,N+1));
    GaussSeidel<Poisson1D, DDeVector> S(A, f, 1.);

    //-- coarse grid -------------------------------------------------------------
    int n = 31;
    Poisson1D                         A_c(n+1);
    DDeVector                         u_c(_(0,n+1)), f_c(_(0,n+1));
    GaussSeidel<Poisson1D, DDeVector> S_c(A_c, f_c, 1.);

    //-- initial guess with two frequencies ------------------------------------
    for (int k=1; k<=N; ++k) {
        double x = k*h;
        u(k) = sin(27*M_PI*x) + sin(3*M_PI*x);
    }

    //-- smooth on fine grid ---------------------------------------------------
    SnapShot snap("res_cg");

    for (int k=1; k<=10; ++k) {
        u = S*u;
    }

    r = f - A*u;
    snap(1) << r << endl;

    //-- solve on coarse grid --------------------------------------------------

    f_c = R*r;
    for (int k=1; k<=300; ++k) {
        u_c = S_c*u_c;
    }

    //-- apply correction ------------------------------------------------------
    u += P*u_c;

    r = f - A*u;
    snap(2) << r << endl;

    //-- smooth again on fine grid ---------------------------------------------
    for (int k=1; k<=10; ++k) {
        u = S*u;
    }

    r = f - A*u;
    snap(3) << r << endl;

}












