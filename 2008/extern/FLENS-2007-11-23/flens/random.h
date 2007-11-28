#ifndef FLENS_RANDOM_H
#define FLENS_RANDOM_H 1

#include <flens/array.h>
#include <flens/densevector.h>
#include <flens/resultclosures.h>

namespace flens {

enum Distribution { Uniform = 0 };

template <Distribution dist, typename T>
T
randomValue();

template <Distribution Dist>
    struct random_;

template <Distribution dist>
    ResultClosure<random_<dist> >
    random(int numRows=0, int numCols=0);

} // namespace flens

#include <flens/random.tcc>

#endif // FLENS_RANDOM_H













