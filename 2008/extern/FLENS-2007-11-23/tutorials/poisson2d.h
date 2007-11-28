#ifndef TUTORIALS_POISSON2D_H
#define TUTORIALS_POISSON2D_H 1

#include <flens/flens.h>

namespace flens {
    
class Poisson2D;

template <>
struct TypeInfo<Poisson2D>
{
    typedef Poisson2D   Impl;
    typedef double      ElementType;
};

class Poisson2D
    : public SymmetricMatrix<Poisson2D>
{
    public:
        Poisson2D() {}
    
        Poisson2D(int _rh) : rh(_rh) {}
    
        int rh;
};

} // namespace flens

#endif // TUTORIALS_POISSON2D_H
