#ifndef FLENS_RESULTCLOSURE_H
#define FLENS_RESULTCLOSURE_H 1

#include <flens/matvec.h>
#include <flens/storage.h>

namespace flens {

template <typename Function>
class ResultClosure
    : public Vector<ResultClosure<Function> >,
      public Matrix<ResultClosure<Function> >
{
    public:
        ResultClosure(const Function &function);

        template <typename LHS>
            void
            copyTo(LHS &lhs) const;

    private:
        const Function _function;
};

template <typename I>
struct TypeInfo<ResultClosure<I> >
{
    typedef void ElementType;
    typedef ResultClosure<I> Impl;
};

template <typename I, typename LHS>
void
copy(const ResultClosure<I> &rhs, LHS &lhs);

//------------------------------------------------------------------------------

template <typename First, typename Second, typename Third>
struct Triple
{
    Triple(First &first, Second &second, Third &third);

    template <typename Function>
        void
        operator=(const ResultClosure<Function> &rhs);

    First &first;
    Second &second;
    Third &third;
};

//------------------------------------------------------------------------------

template <typename First, typename Second>
struct Pair
{
    Pair(First &first, Second &second);

    template <typename Function>
        void
        operator=(const ResultClosure<Function> &rhs);

    template <typename Third>
        Triple<First, Second, Third>
        operator,(Third &third)
        {
            return Triple<First, Second, Third>(first, second, third);
        }

    First &first;
    Second &second;
};


//------------------------------------------------------------------------------

template <typename First, typename Second>
Pair<First, Second>
operator,(First &first, Second &second);

} // namespace flens

#include <flens/resultclosures.tcc>

#endif // FLENS_RESULTCLOSURE_H













