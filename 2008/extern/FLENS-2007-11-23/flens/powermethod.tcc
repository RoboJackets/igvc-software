#include <cstdlib>
#include <flens/cg.h>
#include <flens/random.h>
#include <flens/vectornorm.h>

namespace flens {

template <typename MatA, typename VecX>
void
powerMethod(const MatA &A, typename MatA::ElementType tol,
            double &lambda, VecX &y)
{
    int maxIterations = std::numeric_limits<int>::max();

    lambda = 0.;
    double lambdaOld = 0.;
    VecX z = y;
    for (int i=1; i<=maxIterations; ++i) {
        lambdaOld = lambda;
        y = A*z;
        lambda = norm<lInfinity>(y);
        if (fabs(lambda-lambdaOld)<tol*lambda) {
            lambda = (y*z)/(z*z);
            y = z;
            return;
        }
        z = y / lambda;
    }
}

template <typename Prec, typename MatA, typename VecX>
void
powerMethod(const Prec &P, const MatA &A, typename MatA::ElementType tol,
            double &lambda, VecX &y)
{
    int maxIterations = std::numeric_limits<int>::max();

    lambda = 0.;
    double lambdaOld = 0.;
    VecX z = y, w;
    for (int i=1; i<=maxIterations; ++i) {
        lambdaOld = lambda;
        y = A*z;
        w = P*y;
        y = w;
        lambda = norm<l2>(y);
        if (fabs(lambda-lambdaOld)<tol*lambda) {
            lambda = (y*z)/(z*z);
            y = z;
            return;
        }
        z = y / lambda;
    }
}

template <typename MatA, typename VecX>
void
inversePowerMethod(const MatA &A, typename MatA::ElementType tol,
                   double &lambda, VecX &y)
{
    int maxIterations = std::numeric_limits<int>::max();

    VecX v, q;
    for (int i=1; i<=maxIterations; ++i) {
        v = y / norm<l2>(y);
        cg(A,y,v,tol);
        lambda = v*y;
        if (norm<l2>(q = -lambda*v+y) <= tol*std::fabs(lambda)) {
            lambda = 1./lambda;
            return;
        }
    }
}

template <typename Prec, typename MatA, typename VecX>
void
inversePowerMethod(const Prec &P, const MatA &A, typename MatA::ElementType tol,
                   double &lambda, VecX &y)
{
    int maxIterations = std::numeric_limits<int>::max();

    VecX v, q;
    for (int i=1; i<=maxIterations; ++i) {
        v = y / norm<l2>(y);
        cg(P,A,y,v,tol);
        lambda = v*y;
        if (norm<l2>(q = -lambda*v+y) <= tol*std::fabs(lambda)) {
            lambda = 1./lambda;
            return;
        }
    }
}

template <typename MatA>
double
condition(const MatA &A, double tol)
{
    DenseVector<Array<double> > ev(A.numRows(), 0);
    ev = random<Uniform>();
    double lambdaMax, lambdaMin;

    powerMethod(A, tol, lambdaMax, ev);
    ev = random<Uniform>();
    inversePowerMethod(A, tol, lambdaMin, ev);

    assert(lambdaMin);
    return lambdaMax / lambdaMin;
}

template <typename Prec, typename MatA>
double
condition(const Prec &P, const MatA &A, double tol)
{
    DenseVector<Array<double> > ev(A.numRows(), 0);
    ev = random<Uniform>();
    double lambdaMax, lambdaMin;

    powerMethod(P, A, tol, lambdaMax, ev);
    ev = random<Uniform>();
    inversePowerMethod(P, A, tol, lambdaMin, ev);

    assert(lambdaMin);
    return lambdaMax / lambdaMin;
}

} // namespace flens












