#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <flens/flens.h>
#include <flens/inv.h>

#include <fftw3.h>

#include <gsl/gsl_cdf.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_rng.h>

#include "Gnuplot.h"

#define REAL(z,i) ((z)[2*(i)])
#define IMAG(z,i) ((z)[2*(i)+1])

using namespace std;
using namespace flens;
using namespace gnuplot;

typedef GeMatrix<FullStorage<double,ColMajor> >  GEMatrix;
typedef SpMatrix<PackedStorage<double,ColMajor,Upper> >  SPMatrix;
typedef DenseVector<Array<double> > DEVector;



int main(void) {



#if 0
	int size = 1000;
	DenseVector<Array<fftw_complex> > in(size, 0);
	DEVector out(size, 0);

	in(2)[0] = 1;

	fftw_plan p = fftw_plan_dft_c2r_1d(size, in.data(), out.data(), FFTW_ESTIMATE);

	fftw_execute(p);
	fftw_destroy_plan(p);

	Gnuplot plotter;
	plotter.plot(out);
#endif

#if 0
	DEVector x(2);
	DEVector y(2);
	GEMatrix A(2);

	x = 2, 1;
	A = 1, 2,
		2, 4;
	cout << "x = " << x << endl;
	cout << "A = " << A << endl;

	y = A * x;
	cout << "y = " << y << endl;
	cout << "x = " << x << endl;

	double z = x * y;
	cout << "z = " << z << endl;
#endif
	return(0);
}




