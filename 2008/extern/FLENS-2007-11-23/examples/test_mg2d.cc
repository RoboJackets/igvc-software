#include <cassert>
#include <cmath>
#include <flens/flens.h>
#include <iostream>

#include <examples/poisson2d.h>

using namespace flens;
using namespace std;

void
setRHS(GridVector2D &f)
{
    GridVector2D::Grid &F = f.grid;

    int n = F.lastRow()-1;
    double h = 1./(n+1);

    for (int i=1; i<=n; ++i) {
        for (int j=1; j<=n; ++j) {
            double x = i*h;
            double y = j*h;

            //F(i,j) = 2*((1-6*x*x)*y*y*(1-y*y) + (1-6*y*y)*x*x*(1-x*x));
            F(i,j) = 2*y*(1-y) + 2*x*(1-x);
            //F(i,j) = -(2-12*x*x)*(y*y*y*y - y*y) - (x*x - x*x*x*x)*(12*y*y -2);
        }
    }
}

void
exactSolution(GridVector2D &x)
{
    GridVector2D::Grid &X = x.grid;

    int n = X.lastRow()-1;
    double h = 1./(n+1);

    for (int i=1; i<=n; ++i) {
        for (int j=1; j<=n; ++j) {
            double x = i*h;
            double y = j*h;

            //X(i,j) = (x*x - x*x*x*x)*(y*y*y*y - y*y);
            X(i,j) = x*(1-x)*y*(1-y);
        }
    }
}

double
errorNorm(const GridVector2D &e, const GridVector2D &x)
{
    const GridVector2D::Grid &E = e.grid;
    const GridVector2D::Grid &X = x.grid;

    int n = X.lastRow()-1;
    double h = 1./(n+1);

    double r = 0;
    for (int i=X.firstRow(); i<=X.lastRow(); ++i) {
        for (int j=X.firstCol(); j<=X.lastCol(); ++j) {
            r += (E(i,j)-X(i,j))*(E(i,j)-X(i,j));
        }
    }
    return std::sqrt(h*r);
}

//------------------------------------------------------------------------------

const long l = 9;
const int  n = (1<<l) - 1;  // n = 2^l -1

GridVector2D  u[l+1], f[l+1], r[l+1];
Poisson2D     A[l+1];

void
initMultigrid()
{
    typedef GridVector2D Vec;

    int N = n;
    for (int i=l; i>=1; --i, N/=2) {
        u[i] = Vec(N+1);
        f[i] = Vec(N+1);
        r[i] = Vec(N+1);
        A[i] = Poisson2D(N+1);
    }
}

Restriction  R;
Prolongation P;

template <typename Smoother>
void
multigrid(int l, int v1, int v2)
{
    Smoother S(A[l], f[l], 1.);

    if (l==2) {
        u[l] = S*u[l];
        u[l] = S*u[l];
    } else {
        for (int v=1; v<=v1; ++v) {
            u[l] = S*u[l];
        }
        r[l] = f[l] - A[l]*u[l];

        f[l-1] = R*r[l];

        u[l-1] = 0;
        multigrid<Smoother>(l-1,v1,v2);
        u[l] += P*u[l-1];
        for (int v=1; v<=v2; ++v) {
            u[l] = S*u[l];
        }
    }
}


//------------------------------------------------------------------------------

GridVector2D e(n+1);

void
printStat(int iteration)
{
    double rNorm = norm(r[l]);

    double error = errorNorm(e,u[l]);

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

    cout << "n = " << n << endl;

    r[l] = f[l] - A[l]*u[l];
    printStat(0);

    multigrid<GaussSeidel_RedBlack<Poisson2D, GridVector2D> >(l,0,1);
    SnapShot snap("mg2D_sol");

    printStat(1);
    //snap(0) << u[l];
    for (int k=2; k<=20; ++k) {
        multigrid<GaussSeidel_RedBlack<Poisson2D, GridVector2D> >(l,1,1);
        printStat(k);
        //snap(k) << u[l];
    }
}












