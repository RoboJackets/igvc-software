
#include <flens/flens.h>
//#include "test.h"
#include "KalmanFilter.h"
#include "Gnuplot.h"


using namespace flens;
using namespace gnuplot;

GEMatrix stateModel(double deltaT){
	GEMatrix A(2,2);
	A = 1, 0,
		0, 1;
	return(A);
}

GEMatrix controlModel(double){
	GEMatrix B(2,2);
	B = 1, 0,
		0, 1;
	return(B);
}

GEMatrix measurementModel(double){
	GEMatrix C(2,2);
	C = 1, 0,
		0, 1;
	return(C);
}

int main(void) {

	int k = 10000;

	CovMatrix processNoise(2,2);
	processNoise =  0, 0,
					0, 0;
	CovMatrix measureNoise(2,2);
	measureNoise =  0, 0,
					0, 0;

	DenseVector<Array<control> > x(k);
	control temp(2);
	for(int i = 1; i <= k; i++) {
		temp = sin(.1 * double(i)),
				cos(.5 * double(i)); // it would be nicer if there were a direct way to do this
		x(i) = temp;
	}
	DenseVector<Array<control> > u(k);
	for(int i = 1; i < k; i++) {
		u(i) =  x(i+1) - x(i);
	}
	DenseVector<Array<measurement> > z(k);
	for(int i = 1; i <= k; i++) {
		z(i).resize(2);
		z(i) =  x(i);
	}

	StateVector muInital(2);
	muInital = x(1);
	CovMatrix SigmaInital(2,2);
	SigmaInital = 1, 0,
				  0, 1;

	KalmanFilter testFilter(muInital, SigmaInital,
							stateModel, controlModel, measurementModel,
							processNoise, measureNoise);

	DenseVector<Array<StateVector> > StateEstimateArray(k);
	DenseVector<Array<CovMatrix> >  CovarianceArray(k);

	for(int i = 1; i < k; i++) {
		testFilter.update(u(i), z(i), 1);
		StateEstimateArray(i) = testFilter.stateEstimate();
		CovarianceArray(i) = testFilter.covariance();
	}

	StateVector StateError(2);
	DenseVector<Array<double> > y(k);

	for(int i = 1; i < k; i++) {
		StateError = x(i+1) - StateEstimateArray(i);
		y(i) = StateError(1);
	}

	Gnuplot plotter;
	//plotter.execute("plot 'test.dat' using linespoints");
	plotter.plot(y);

	return(0);
}
