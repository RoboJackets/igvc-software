#include <flens/flens.h>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace flens;
using namespace std;

typedef GeMatrix<FullStorage<double, ColMajor> > DGeMatrix;

int N = 30,g;
double h = 1./N;
double deltat = h*h*0.5,
       c = 1,
       Pi=3.1415;

void
rhs(DGeMatrix &x, DGeMatrix &f, double &time)
{
    for (int i=0; i<=N+1; ++i) {
        for (int j=0; j<=N+1; ++j) {
            double u = i*h, v = j*h;
            if( ((u<=0.25) & (v<=0.25)) | ((u>=0.75) & (v>=0.75)) )
            x(i,j) = -0.5*cos(2*Pi/0.25*u)-0.5*cos(2*Pi/0.25*v)+2;
            else
            x(i,j) = 0;
            f(i,j) = 0;
        }
    }
}

void
explicitStep(DGeMatrix &x, const DGeMatrix &f, DGeMatrix &x1, DGeMatrix &x0)
{
    for (int i=0; i<=N+1; ++i) {
        x(i,1) = 0.0;
        x(i,N) = 0.0;
    }
    for (int j=0; j<=N+1; ++j) {
        x(1,j) = 0.0;
        x(N,j) = 0.0;
    }

    for(int i=0; i<=N+1; i++) {
        for(int j=0; j<=N+1; ++j) {
           x0(i,j) = x1(i,j);
           x1(i,j) = x(i,j);
        }
    }

    for (int i=1; i<=N; ++i) {
        for (int j=1; j<=N; ++j) {
            x(i,j) = c*(deltat*deltat)/(h*h)*(x1(i+1,j)+x1(i-1,j)-4*x1(i,j)+x1(i,j+1)+x1(i,j-1))+2*x1(i,j)-x0(i,j);
        }
    }
}

double
error(const DGeMatrix &x, const DGeMatrix &sol)
{
    double err = 0.0;

    for (int i=1; i<=N; ++i) {
        for (int j=1; j<=N; ++j) {
            double diff = sol(i,j)-x(i,j);
            if ( diff > err ) {
                err = diff;
            }
        }
    }

    return err;
}

void
dumpData(double &time, const DGeMatrix &x)
{
   char filename[250];
    sprintf(filename,"welle2d-%05i.out",g);
    ofstream out(filename);

    for (int i=1; i<=N; ++i) {
             out << " " << endl;
        for (int j=1; j<=N; ++j) {
            out << time << "\t" << i*h << "\t" << j*h << "\t" << x(i,j) << endl;
        }
    }
    out << " " << endl;
}

int
main()
{
    DGeMatrix f(_(0,N+1),_(0,N+1)),
              x(_(0,N+1),_(0,N+1)),
              sol(_(0,N+1),_(0,N+1)),
              x1(_(0,N+1),_(0,N+1)),
              x0(_(0,N+1),_(0,N+1));


    double time = 0.0,
           dumpDeltaT = 0.006,
           dumpTime   = 0.006;

    rhs(x, f, time);
    dumpData(time, x);
    for(int i=0; i<=N+1; i++){
       for(int j=0; j<=N+1; j++){
          x0(i,j)=0;
          x1(i,j)=x(i,j);
       }
    }
    while ( time <= 1.0 ) {
          explicitStep(x, f, x1, x0);
          cout << "time = ";
          cout.width(20);
          cout.precision(6);
          cout << time << endl;

          time += deltat;
          if (time > dumpTime) {
            dumpData(time, x);
            dumpTime += dumpDeltaT;
          }
          g +=1;
  }

    cout << "x = " << x << endl;
    cout << "sol = " << sol << endl;

    return 0;
}













