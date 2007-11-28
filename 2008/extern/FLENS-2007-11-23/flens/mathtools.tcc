#include <cmath>
#include <limits>

namespace flens {

template <typename E>
typename E::ElementType
absmax(const DenseVector<E> &v)
{
        typename E::ElementType max = std::abs(v(v.firstIndex()));
        for (int i=v.firstIndex(); i<=v.lastIndex(); ++i) {
                if (std::abs(v(i)) > max) {
                        max = std::abs(v(i));
                }
        }
        return max;
}

template <typename T>
T
notANumber()
{
    return std::numeric_limits<T>::quiet_NaN();
}

} // namespace flens












