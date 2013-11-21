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
    GaussianVariable(T value = 0, double variance = 0)
        : value(value),
          variance(variance)
    {
    }

    T Value() { return value; }
    double Variance() { return variance; }

    void SetValue(T val) { value = val; }
    void SetVariance(double val) { variance = val; }

    double ProbabilityOfValue(T val)
    {
        using namespace std;
        return (1/sqrt(2*M_PI*variance))*exp( ( -0.5*(val-value)*(val-value) ) / variance );
    }

private:
    T value;
    double variance;
};

#endif // GAUSSIANVARIABLE_H
