#include <flens/random.h>

#include <cstdlib>

namespace flens {

template <>
double
randomValue<Uniform, double>()
{
    return std::rand() / double(RAND_MAX);
}

template <>
std::complex<double>
randomValue<Uniform, std::complex<double> >()
{
    return std::complex<double>(std::rand() / double(RAND_MAX),
                                std::rand() / double(RAND_MAX));
}

} // namespace flens












