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
    for (int k=0; k<=1; ++k) {
        x = GS*x;
    }
}

} // namespace flens

using namespace flens;
using namespace std;

typedef DenseVector<Array<double> > DDeVector;

//-- Problem setup -------------------------------------------------------------

// for   -u'' = f
void
setRHS(DDeVector &f)
{
    int n = f.lastIndex()-1;
    double h = 1./(n+1);

    for (int k=1; k<=n; ++k) {
        f(k) = M_PI*M_PI*sin(M_PI*k*h);
    }
}

void
exactSolution(DDeVector &u)
{
    int n = u.lastIndex()-1;
    double h = 1./(n+1);

    for (int k=0; k<=n+1; ++k) {
        u(k) = sin(M_PI*k*h);
    }
}

//-- Multigrid -----------------------------------------------------------------

const long l = 12;
const int  n = (1<<l) - 1;  // n = 2^l -1
DDeVector  u[l+1], f[l+1], r[l+1];
Poisson1D  A[l+1];


void
initMultigrid()
{
    typedef DDeVector Vec;

    int N = n;
    for (int i=l; i>=1; --i, N/=2) {
        u[i] = Vec(_(0,N+1));
        f[i] = Vec(_(0,N+1));
        r[i] = Vec(_(0,N+1));
        A[i] = Poisson1D(N+1);
    }
}

Restriction  R;
Prolongation P;

DDeVector r1, r2, u1, u2;


void
init(DDeVector &x)
{
    for (int i=x.firstIndex(); i<=x.lastIndex(); ++i) {
        x(i) = 0;
    }
}

template <typename S>
void
multigrid(int l, int v1, int v2)
{
    if (l==2) {
        u[l] = S(A[l], f[l], 1.)*u[l];
        u[l] = S(A[l], f[l], 1.)*u[l];
    } else {
        for (int v=1; v<=v1; ++v) {
            u[l] = S(A[l], f[l], 1.)*u[l];
        }
        r[l] = f[l] - A[l]*u[l];
        f[l-1] = R*r[l];

        u[l-1] = 0;
        multigrid<S>(l-1,v1,v2);
        u[l] += P*u[l-1];
        for (int v=1; v<=v2; ++v) {
            u[l] = S(A[l], f[l], 1.)*u[l];
        }
    }
}


DDeVector e(_(0,n+1));

void
printStat(int iteration)
{
    double rNorm = 0;
    for (int i=1; i<=n; ++i) {
        rNorm += r[l](i)*r[l](i);
    }
    rNorm = sqrt(rNorm);

    double error = 0;
    for (int i=1; i<=n; ++i) {
        error += (u[l](i)-e(i))*(u[l](i)-e(i));
    }
    error = sqrt(error);


    cout.width(3);
    cout << iteration << ") | ";

    cout.precision(12);
    cout.setf(std::ios::fixed);
    cout.width(17);
    cout << rNorm << " | ";

    cout.precision(12);
    cout.setf(std::ios::fixed);
    cout.width(16);
    cout << error << " | " << endl;
}


int
main()
{
    initMultigrid();
    setRHS(f[l]);
    exactSolution(e);

    r[l] = f[l] - A[l]*u[l];
    multigrid<GaussSeidel<Poisson1D, DDeVector> >(l,0,1);
    SnapShot snap("mg_sol");

    cout << "n = " << n << endl;
    printStat(0);
    snap(0) << u[l];
    for (int k=1; k<=20; ++k) {
        multigrid<GaussSeidel<Poisson1D, DDeVector> >(l,1,1);
        printStat(k);
        snap(k) << u[l];
    }
}












