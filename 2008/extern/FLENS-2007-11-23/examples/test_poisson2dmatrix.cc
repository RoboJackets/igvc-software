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

    double r = 0;
    for (int i=X.firstRow(); i<=X.lastRow(); ++i) {
        for (int j=X.firstCol(); j<=X.lastCol(); ++j) {
            r += (E(i,j)-X(i,j))*(E(i,j)-X(i,j));
        }
    }
    return std::sqrt(r);
}

int
main()
{
    int N = 15;
    Poisson2D A(N+1);
    Restriction R;
    Prolongation P;

    GridVector2D u(N+1), f(N+1), e(N+1), r(N+1);
    GaussSeidel<Poisson2D, GridVector2D> S(A, f, 1.7);

    int n = 7;
    Poisson2D A_c(n+1);

    GridVector2D u_c(n+1), f_c(n+1);
    GaussSeidel<Poisson2D, GridVector2D> S_c(A_c, f_c, 1.7);


    setRHS(f);
    exactSolution(e);

    for (int k=1; k<=5; ++k) {
        u = S*u;
    }
    r = f - A*u;
    cout << "r = " << norm(r) << "  ";
    cout << "e = " << errorNorm(e, u) << endl;

    f_c = R*r;
    for (int k=1; k<=50; ++k) {
        u_c = S_c*u_c;
    }


    u += P*u_c;
    r = f - A*u;
    cout << "r = " << norm(r) << "  ";
    cout << "e = " << errorNorm(e, u) << endl;

    for (int k=1; k<=5; ++k) {
        u = S*u;
    }
    r = f - A*u;
    cout << "r = " << norm(r) << "  ";
    cout << "e = " << errorNorm(e, u) << endl;




    //cout << "f = " << F << endl;
    //cout << "x = " << X << endl;
    //cout << "e = " << E << endl;
}












