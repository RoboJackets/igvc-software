#ifndef GAUSSIANVARIABLE_H
#define GAUSSIANVARIABLE_H

#include <cmath>

/*
 * Represents a Gaussian Distribution with mean = value and sigma^2 = variance.
 */
template <class T>
class GaussianVariable
{
public:
    T Value;
    double Variance;

    GaussianVariable(T value = 0, double variance = 0)
        : Value(value),
          Variance(variance)
    {
    }

    double ProbabilityOfValue(T val)
    {
        using namespace std;
        return (1/sqrt(2*M_PI*Variance))*exp( ( -0.5*(val-Value)*(val-Value) ) / Variance );
    }

    GaussianVariable& operator=(T val)
    {
        Value = val;
        return *this;
    }
    operator T()
    {
        return Value;
    }

    operator T() const
    {
        return Value;
    }

private:
};

#endif // GAUSSIANVARIABLE_H
