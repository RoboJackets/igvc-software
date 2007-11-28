#define ASSERT(x) throwException(x)

#include <flens/flens.h>

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
run(void (*test)(), bool expectedResult)
{
    bool result = true;
    
    try {
        test();
    }
    catch (int &i) {
        result = false;
    }
    if (result==expectedResult) {
        std::cerr << "passed." << endl;
    } else {
        std::cerr << "failed." << endl;
    }
}

void
test1()
{
    DenseVector<Array<double> > x(3), y(3);
    y = 3, 2, 1;
    x = 1, 2, 3;

    y = y + x;
}

void
test2()
{
    DenseVector<Array<double> > x(3), y(3);
    y = 3, 2, 1;
    x = 1, 2, 3;

    y = x + 0.5*y;
}

int
main()
{
    cerr << "test1 "; run(test1, true);
    cerr << "test2 "; run(test2, false);
}