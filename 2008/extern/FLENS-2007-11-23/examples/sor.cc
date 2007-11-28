#include <flens/flens.h>
#include <cmath>
#include <iostream>

using namespace flens;
using namespace std;

typedef DenseVector<Array<double> > DeVector;

int N = 7;
double h = 1./(N+1);
double omega = 1.7;

void
rhs(DeVector &f)
{
    for (int i=0; i<=N+1; ++i) {
        double x = i*h;
        f(i) = 2*x-1;
    }
    f(0) /= 2;
    f(N+1) /= 2;
}

void
sorStep(DeVector &x, const DeVector &f)
{
    double c = omega;
    x(0) = x(0)*(1-omega)+c*(x(1)+h*h*f(0));

    c = omega/2;
    for (int i=1; i<=N; ++i) {
        x(i) = x(i)*(1-omega)+c*(x(i-1)+x(i+1)+h*h*f(i));
    }

    c = omega;
    x(N+1) = x(N+1)*(1-omega)+c*(x(N)+h*h*f(N+1));

    double sum = 0;
    for (int i=0; i<=N+1; ++i) {
        sum += x(i);
    }
    for (int i=0; i<=N+1; ++i) {
        x(i) -= sum/(N+2);
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

    res = sqr(rh*rh*(x(0)-x(1)) - b(0));

    for (int i=1; i<=N; ++i) {
        res += sqr(rh*rh*(-x(i-1)+2*x(i)-x(i+1)) -b(i));
    }

    res += sqr(rh*rh*(-x(N)+x(N+1))-b(N+1));
    return sqrt(res);
}

int
main()
{
    DeVector f(_(0,N+1)),
             x(_(0,N+1));

    rhs(f);

    double res = 0;
    int iteration = 1;
    while ((res=residuum(x,f))>0.00001) {
        sorStep(x, f);
        cout << "iteration = ";
        cout.width(4);
        cout << iteration;
        cout << ", residuum = ";
        cout.width(20);
        cout.precision(6);
        cout << res << endl;
        ++iteration;
    }
    cout << "x = " << x << endl;

    return 0;
}












