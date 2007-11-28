#define ASSERT(x) throwException(x)

#include <flens/flens.h>
#include <string>
#include <limits>

using namespace flens;
using namespace std;

void
throwException(bool b)
{
    if (!b) {
        throw int(1);
    }
}

void
run(const string &s, void (*test)(), bool expectedResult)
{
    bool result = true;

    try {
        test();
    }
    catch (int &i) {
        result = false;
    }
    if (result==expectedResult) {
        std::cerr << s << " passed." << endl;
    } else {
        std::cerr << s << " !!! FAILED !!!" << endl;
    }
}

template <typename VX, typename VY>
void
assertEqual(const DenseVector<VX> &x, const DenseVector<VY> &y)
{
    double eps = numeric_limits<double>::epsilon();
    typename DenseVector<VX>::NoView diff;

    diff = x - y;
    for (int i=diff.firstIndex(); i<=diff.lastIndex(); ++i) {
        ASSERT(abs(diff(i))<eps);
    }
}
