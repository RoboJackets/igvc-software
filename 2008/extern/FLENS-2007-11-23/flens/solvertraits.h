#ifndef FLENS_SOLVERTRAITS_H
#define FLENS_SOLVERTRAITS_H 1

namespace flens {

template <typename AnyType, typename ResultType>
struct SolverRefTrait
{
    typedef const ResultType Type;
};

template <typename ResultType>
struct SolverRefTrait<ResultType, ResultType>
{
    typedef const ResultType &Type;
};

} // namespace flens

#endif // FLENS_SOLVERTRAITS_H












