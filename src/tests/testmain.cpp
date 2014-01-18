#include <QTest>
#include <tests/teststringutils.hpp>
#include <tests/testlinedetection.hpp>

int main()
{
    QTest::qExec(new TestStringUtils());
    QTest::qExec(new TestLineDetection());
}
