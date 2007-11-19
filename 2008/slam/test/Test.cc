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

#include "NormalPD.h"
//#include "NormalPD.cc"
#include "Gnuplot.h"

#define REAL(z,i) ((z)[2*(i)])
#define IMAG(z,i) ((z)[2*(i)+1])

using namespace std;
using namespace flens;

typedef GeMatrix<FullStorage<double,ColMajor> >  GEMatrix;
typedef SpMatrix<PackedStorage<double,ColMajor,Upper> >  SPMatrix;
typedef DenseVector<Array<double> > DEVector;

typedef DenseVector<Array<double> > MeanVector;
typedef GeMatrix<FullStorage<double,ColMajor> > CovMatrix;

int main(void) {


	DEVector y(10);
	y = 1, 3, 5, 7, 2, 3, 4, 9, 5, 0;
	Gnuplot windowA;
	windowA.plot(y);
	windowA.title("Woot");
	windowA.xlabel("x");
	windowA.axis(-1, 11);

#if 0
	N = 2;
	MeanVector mu(N);
	CovMatrix sigma(N,N);
	mu = 0, 0;
	sigma = 1, 0,
			0, 1;

	DenseVector<Array<Probability> > result(N);
	CovMatrix tempM(N);
	MeanVector tempV1(N);
	MeanVector tempV2(N);
	E tempVal;

	tempM = inv(sigma);
	tempV1 = value - mu;
	tempV2 = tempM * tempV1;
	tempVal = tempV1 * tempV2;
	result = (1/(pow((2*M_PI),(N/2))*sqrt(det(sigma)))) * exp(-tempVal/2);
#endif

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

	return(0);
}

