#include <flens/flens.h>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace flens;
using namespace std;

typedef DenseVector<Array<double> > DeVector;

int N = 198;
double h = 1./(N+1);
double omega = 1.7;

void
rhs(DeVector &f)
{
    for (int i=0; i<=N+1; ++i) {
        double x = i*h;
        f(i) = 3*x;
    }
}

void
solution(DeVector &sol)
{
    for (int i=0; i<=N+1; ++i) {
        double x = i*h;
        sol(i) = -0.5*x*x*x + 0.5*x;
    }
}

void
sorStep(DeVector &x, const DeVector &f)
{
    x(0) = 0;
    x(N+1) = 0;

    double c = omega/2;
    for (int i=1; i<=N; ++i) {
        x(i) = x(i)*(1-omega)+c*(x(i-1)+x(i+1)+h*h*f(i));
    }
}

double
sqr(double x)
{
    return x*x;
}

double
residuum(const DeVector &x, const DeVector &b)
{
    double res = 0;
    double rh = N+1;

    for (int i=1; i<=N; ++i) {
        res += sqr(rh*rh*(-x(i-1)+2*x(i)-x(i+1)) -b(i));
    }

    return sqrt(res);
}

double
error(const DeVector &x, const DeVector &sol)
{
    double err = 0;

    for (int i=1; i<=N; ++i) {
        double diff = abs(sol(i)-x(i));
        if ( diff > err ) {
            err = diff;
        }
    }

    return err;
}

void
dumpData(int &iteration, const DeVector &x)
{
    char filename[250];
    sprintf(filename,"gs-1d-%05i.out",iteration);
    ofstream out(filename);

    for (int i=0; i<=N+1; ++i) {
        out << i*h << "\t" << x(i) << endl;
    }
}

void
dumpSol(const DeVector &sol)
{
    ofstream out("gs-1d-sol.out");

    for (int i=0; i<=N+1; ++i) {
        out << i*h << "\t" << sol(i) << endl;
    }
}

int
main()
{
    DeVector f(_(0,N+1)),
             x(_(0,N+1)),
             sol(_(0,N+1));

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












