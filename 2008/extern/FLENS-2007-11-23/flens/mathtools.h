#ifndef FLENS_MATHTOOLS_H
#define FLENS_MATHTOOLS_H 1

#include <flens/densevector.h>

namespace flens {

int
factorial(int n);

template <typename E>
typename E::ElementType
absmax(const DenseVector<E> &v);

int
floor2j(int x);

int
ceil2j(int x);

template <typename T>
T
notANumber();

double
pow(int base, int exp);

double
sqrt(int x);

int
ifloor(double x);

int
iceil(double x);

DenseVector<Array<double> >
linspace(double from, double to, int numTicks);

} // namespace flens

#include <flens/mathtools.tcc>

#endif // FLENS_MATHTOOLS_H












