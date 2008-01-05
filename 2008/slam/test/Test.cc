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

#if 0
	DEVector y(10);
	y = 1, 3, 5, 7, 2, 3, 4, 9, 5, 0;
	Gnuplot windowA;
	windowA.plot(y);
	windowA.title("Woot");
	windowA.xlabel("x");
	windowA.axis(-1, 11);
#endif

	int N = 2;
	int size = 10;
	MeanVector mu(N);
	CovMatrix sigma(N,N);
	mu = 0, 0;
	sigma = 1, 0,
			0, 1;

	CovMatrix sigmaTRF(N,N);
	MeanVector tempV1(N);
	MeanVector tempV2(N);
	double tempVal;

	DenseVector<Array<int> > p(N);
	sigmaTRF = sigma;
	trf(sigmaTRF, p);

	double detSigma = 0;
	for(int i = 1; i <= sigma.numRows(); i++) {
		detSigma += sigmaTRF(i,i);
	}


	CovMatrix invSigma(N,N);
	invSigma = sigma;
	inv(invSigma);

	DenseVector<Array<MeanVector> > value(size * size);
	double deltaX = 1/size; //range divided by size
	double deltaY = 1/size; //range divided by size
	for(int i = 1; i <= size; i++) {
		for(int j = 1; j <= size; j++) {
			int index = (i - 1) * size + j;
			value(index).resize(2);
			value(index)(1) = i;
			value(index)(2) = j;
		}
	}

	DenseVector<Array<Probability> > z(size * size);

	for(int i = 1; i <= (size * size); i++) {
		tempV1 = value(i) - mu;
		tempV2 = invSigma * tempV1;
		tempVal = tempV1 * tempV2;
		z(i) = (1/(pow((2*M_PI),(N/2))*sqrt(detSigma))) * exp(-tempVal/2);
	}

	DEVector x(size * size);
	DEVector y(size * size);
	for(int i = 1; i <= (size * size); i++) {
		x(i) = value(i)(1);
		y(i) = value(i)(2);
	}

	Gnuplot plotter;
	plotter.plot(x, z);

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

