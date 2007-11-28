#ifndef TUTORIALS_GRIDVECTOR2D_H
#define TUTORIALS_GRIDVECTOR2D_H 1

#include <tutorials/poisson2d.h>

namespace flens {

class GridVector2D;

template <>
struct TypeInfo<GridVector2D>
{
    typedef GridVector2D Impl;
    typedef double       ElementType;
};

class GridVector2D
    : public Vector<GridVector2D>
{
    public:
        typedef GeMatrix<FullStorage<double, RowMajor> >  Grid;

        int    rh;
        Grid   grid;

        GridVector2D()
            : rh(0)
        {
        }

        explicit
        GridVector2D(int _rh)
            : rh(_rh), grid(_(0,rh),_(0,rh))
        {
        }
        
        GridVector2D &
        operator=(double value)
        {
            grid = value;
            return *this;
        }
        
        template <typename RHS>
        GridVector2D &
        operator=(const Vector<RHS> &rhs)
        {
            copy(rhs.impl(), *this);
            return *this;
        }
        
        template <typename RHS>
        GridVector2D &
        operator+=(const Vector<RHS> &rhs)
        {
            axpy(1., rhs.impl(), *this);
            return *this;
        }

        template <typename RHS>
        GridVector2D &
        operator-=(const Vector<RHS> &rhs)
        {
            axpy(-1., rhs.impl(), *this);
            return *this;
        }
};

} // namespace flens

#endif // TUTORIALS_GRIDVECTOR2D_H
