#include <cassert>

namespace flens {

template <typename Function>
ResultClosure<Function>::ResultClosure(const Function &function)
    : _function(function)
{
}

template <typename Function>
template <typename LHS>
void
ResultClosure<Function>::copyTo(LHS &lhs) const
{
    _function(lhs);
}

template <typename I, typename LHS>
void
copy(const ResultClosure<I> &rhs, LHS &lhs)
{
    rhs.copyTo(lhs);
}

//------------------------------------------------------------------------------

template <typename First, typename Second>
Pair<First, Second>::Pair(First &f, Second &s)
    : first(f), second(s)
{
}

template <typename First, typename Second>
template <typename Function>
void
Pair<First, Second>::operator=(const ResultClosure<Function> &rhs)
{
    rhs.copyTo(*this);
}

//------------------------------------------------------------------------------

template <typename First, typename Second>
Pair<First, Second>
operator,(First &first, Second &second)
{
    return Pair<First, Second>(first, second);
}

//------------------------------------------------------------------------------

template <typename First, typename Second, typename Third>
Triple<First, Second, Third>::Triple(First &f, Second &s, Third &t)
    : first(f), second(s), third(t)
{
}

template <typename First, typename Second, typename Third>
template <typename Function>
void
Triple<First, Second, Third>
::operator=(const ResultClosure<Function> &rhs)
{
    rhs.copyTo(*this);
}

} // namespace flens












