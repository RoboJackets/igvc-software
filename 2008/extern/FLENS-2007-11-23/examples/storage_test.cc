#include <iostream>

#include <flens/flens.h>

using namespace flens;
using namespace std;

template <typename T, StorageOrder order>
void
init(FullStorage<T, order> &A)
{
    int count = 1;
    for (int i=A.firstRow(); i<A.firstRow()+A.numRows(); ++i) {
        for (int j=A.firstCol(); j<A.firstCol()+A.numCols(); ++j) {
            A(i, j) = count++;
        }
    }
}

template <typename T, StorageOrder order>
void
print(const FullStorage<T, order> &A)
{
    for (int i=A.firstRow(); i<A.firstRow()+A.numRows(); ++i) {
        for (int j=A.firstCol(); j<A.firstCol()+A.numCols(); ++j) {
            cout << A(i, j) << " ";
        }
        cout << endl;
    }
}

int
main()
{
    int m = 5,
        n = 3;
    FullStorage<double, ColMajor> A(m, n), B;

    init(A);
    B = A;
    print(B);
}












