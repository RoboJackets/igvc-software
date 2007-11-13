
#include <flens/flens.h>
//#include "test.h"
#include "KalmanFilter.h"
#include "Gnuplot.h"


using namespace flens;
using namespace gnuplot;

GEMatrix stateModel(double){
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

#if 0
	LinModel stateModel(2,2);
	stateModel = 1, 0,
				 0, 1;
	LinModel controlModel(2,2);
	controlModel =  1, 0,
					0, 1;
	LinModel measurementModel(2,2);
	measurementModel =  1, 0,
						0, 1;
#endif

	CovMatrix processNoise(2,2);
	processNoise =  0, 0,
					0, 0;
	CovMatrix measureNoise(2,2);
	measureNoise =  0, 0,
					0, 0;

	// is there a better way to do this?
	DenseVector<Array<control> > x(k);
	for(int i = 1; i <= k; i++) {
		x(i).resize(2);
		x(i) =  sin(.1 * double(i)),
				cos(.5 * double(i));
	}
	DenseVector<Array<control> > u(k);
	for(int i = 1; i < k; i++) {
		u(i).resize(2);
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
		StateEstimateArray(i).resize(2);
		StateEstimateArray(i) = testFilter.stateEstimate();
		CovarianceArray(i).resize(2,2);
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
