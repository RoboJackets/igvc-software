#ifndef FLENS_POWERMETHOD_H
#define FLENS_POWERMETHOD_H 1

#include <flens/matvec.h>

namespace flens {

template <typename MatA, typename VecX>
    void
    powerMethod(const MatA &A, typename MatA::ElementType tol,
                double &lambda, VecX &y);

template <typename Prec, typename MatA, typename VecX>
    void
    powerMethod(const Prec &P, const MatA &A, typename MatA::ElementType tol,
                double &lambda, VecX &y);

template <typename MatA, typename VecX>
    void
    inversePowerMethod(const MatA &A, typename MatA::ElementType tol,
                       double &lambda, VecX &y);

template <typename Prec, typename MatA, typename VecX>
    void
    inversePowerMethod(const Prec &P, const MatA &A, typename MatA::ElementType tol,
                       double &lambda, VecX &y);

template <typename MatA>
    double
    condition(const MatA &A, double tol=1e-15);


template <typename Prec, typename MatA>
    double
    condition(const Prec &P, const MatA &A, double tol=1e-15);

} // namespace flens

#include <flens/powermethod.tcc>

#endif // FLENS_POWERMETHOD_H












