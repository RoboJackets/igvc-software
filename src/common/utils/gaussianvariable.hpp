#ifndef GAUSSIANVARIABLE_H
#define GAUSSIANVARIABLE_H

#include <cmath>
#include <iostream>

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

    GaussianVariable(const GaussianVariable &other)
        : Value(other.Value),
          Variance(other.Variance)
    {
        std::cout << "copy" << std::endl;
    }

    GaussianVariable(GaussianVariable &&other)
        : Value(other.Value),
          Variance(other.Variance)
    {
        std::cout << "move" << std::endl;
    }

    GaussianVariable& operator=(const GaussianVariable &other)
    {
        Value = other.Value;
        Variance = other.Variance;
        std::cout << "copy assign" << std::endl;
        return *this;
    }

    GaussianVariable& operator=(GaussianVariable &&other)
    {
        Value = other.Value;
        Variance = other.Variance;
        std::cout << "move assignment" << std::endl;
        return *this;
    }

    double ProbabilityOfValue(T val)
    {
        using namespace std;
        return (1/sqrt(2*M_PI*Variance))*exp( ( -0.5*(val-Value)*(val-Value) ) / Variance );
    }

    GaussianVariable& operator=(T val)
    {
        std::cout << "value assignment" << std::endl;
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
