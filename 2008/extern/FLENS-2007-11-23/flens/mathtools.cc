#include <cassert>
#include <limits>

#include <flens/mathtools.h>

namespace flens {

int
factorial(int n)
{
    int fac = 1;
    for (int i=2; i<=n; ++i) {
        fac *= i;
    }
    return fac;
}

int floor2j(int x)
{
    assert(x>=0);

    int result = 0;
    while (x>>=1) {
        ++result;
    };
    return result;
}

int
ceil2j(int x)
{
    assert(x>=0);

    int result=0;
    while(x>(1<<result)) {
        result++;
    }
    return result;
}

double
pow(int base, int exp)
{
    return std::pow(static_cast<double>(base), exp);
}

double
sqrt(int x)
{
    return std::sqrt(static_cast<double>(x));
}

int
ifloor(double x)
{
    return static_cast<int>(std::floor(x));
}

int
iceil(double x)
{
    return static_cast<int>(std::ceil(x));
}

DenseVector<Array<double> >
linspace(double from, double to, int numTicks)
{
    double step = (to-from) / (numTicks-1);
    DenseVector<Array<double> > x(numTicks,0);
    x(0) = from;
    for (int i=1; i<numTicks-1; ++i) {
        x(i) = from + i*step;
    }
    x(x.lastIndex()) = to;
    return x;
}

} // namespace flens












