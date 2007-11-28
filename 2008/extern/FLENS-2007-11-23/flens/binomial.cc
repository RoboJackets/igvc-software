#include <cmath>
#include <flens/binomial.h>

namespace flens {

int
binomial(int n, int k)
{
    if (k>n) {
        return 0;
    }
    if (k==n) {
        return 1;
    }
    if (k==0) {
        return 1;
    }
    double res = 1.0;
    for (int i=k+1; i<=n; ++i) {
        res *= i;
    }
    double den = 1.0;
    for (int i=2; i<=n-k; ++i) {
        den *= i;
    }
    return static_cast<int>(std::floor(res / den));
}

} // namespace flens













