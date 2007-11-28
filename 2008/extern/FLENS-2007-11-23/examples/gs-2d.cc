#include <flens/flens.h>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace flens;
using namespace std;

typedef GeMatrix<FullStorage<double, ColMajor> > DGeMatrix;

int N = 100;
double h = 1./(N+1),
       rh = 1./h;
double omega = 1.7,
       Pi=3.1415;

void
rhs(DGeMatrix &f)
{
    for (int i=0; i<=N+1; ++i) {
        for (int j=0; j<=N+1; ++j) {
            double x = i*h,
                   y = j*h;
            f(i,j) = 8*Pi*Pi*sin(2*Pi*x)*sin(2*Pi*y);
        }
    }
}

void
solution(DGeMatrix &sol)
{
    for (int i=0; i<=N+1; ++i) {
        for (int j=0; j<=N+1; ++j) {
            double x = i*h,
                   y = j*h;
            sol(i,j) = sin(x*2*Pi)*sin(y*2*Pi);
        }
    }
}

void
sorStep(DGeMatrix &x, const DGeMatrix &f)
{
    for (int i=0; i<=N+1; ++i) {
        x(i,0) = 0;
        x(i,N+1) = 0;
    }
    for (int j=0; j<=N+1; ++j) {
        x(0,j) = 0;
        x(N+1,j) = 0;
    }

    double c = h*h*omega/4;
    for (int i=1; i<=N; ++i) {
        for (int j=1; j<=N; ++j) {
            x(i,j) = x(i,j)*(1.-omega)+c*((x(i-1,j)+x(i+1,j)+x(i,j-1)+x(i,j+1))*rh*rh+f(i,j));
        }
    }
}

double
sqr(double x)
{
    return x*x;
}

double
residuum(const DGeMatrix &x, const DGeMatrix &b)
{
    double res = 0;
    double rh = N+1;

    for (int i=1; i<=N; ++i) {
        for (int j=1; j<=N; ++j) {
            res += sqr(rh*rh*(-x(i-1,j)+4*x(i,j)-x(i+1,j)-x(i,j-1)-x(i,j+1)) -b(i,j));
        }
    }

    return sqrt(res);
}

double
error(const DGeMatrix &x, const DGeMatrix &sol)
{
    double err = 0;

    for (int i=1; i<=N; ++i) {
        for (int j=1; j<=N; ++j) {
            double diff = abs(sol(i,j)-x(i,j));
            if ( diff > err ) {
                err = diff;
            }
        }
    }

    return err;
}

void
dumpData(int &iteration, const DGeMatrix &x)
{
    char filename[250];
    sprintf(filename,"gs-2d-%05i.out",iteration);
    ofstream out(filename);

    for (int i=0; i<=N+1; ++i) {
        for (int j=0; j<=N+1; ++j) {
            out << i*h << "\t" << j*h << "\t" << x(i,j) << endl;
        }
    }
}

void
dumpSol(const DGeMatrix &sol)
{
    ofstream out("gs-2d-sol.out");

    for (int i=0; i<=N+1; ++i) {
        for (int j=0; j<=N+1; ++j) {
            out << i*h << "\t" << j*h << "\t" << sol(i,j) << endl;
        }
    }
}

int
main()
{
    DGeMatrix f(_(0,N+1),_(0,N+1)),
             x(_(0,N+1),_(0,N+1)),
             sol(_(0,N+1),_(0,N+1));

    rhs(f);
    solution(sol);

    dumpSol(sol);

    double res = 0;
    int iteration = 1;
    while ((res=residuum(x,f))>0.00001) {
        sorStep(x, f);
        if ( (iteration%200==0) || ((iteration%25==0)&&(iteration<=100)) ) {
            dumpData(iteration, x);
        }
        cout << "iteration = ";
        cout.width(4);
        cout << iteration;
        cout << " residuum = ";
        cout.width(20);
        cout.precision(6);
        cout << res;
        cout << " fehler = ";
        cout.width(20);
        cout.precision(6);
        cout << error(x,sol) << endl;
        ++iteration;
    }

    cout << "x = " << x << endl;
    cout << "sol = " << sol << endl;

    return 0;
}












