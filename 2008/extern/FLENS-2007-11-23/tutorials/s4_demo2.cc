#include <fstream>
#include <iostream>
#include <cmath>

using namespace std;

int
main()
{
    int n = 100;
    double h = 1./(n+1);

    ofstream file("s4_somedata.dat");

    for (int i=0; i<=n; ++i) {
        for (int j=0; j<=n; ++j) {
            double x = j*h;
            double y = i*h;
            file.width(12);
            file << x << " " << y << " "
                 << pow(x,1.6)*pow(y,0.2) + 1./(1+exp(y*x))
                 << std::endl;
        }
        file << std::endl;
    }
}
