
#include <flens/flens.h>
//#include "test.h"
#include "KalmanFilter.h"
#include "Gnuplot.h"

using namespace flens;
using namespace gnuplot;

int main(void) {

#if 0
	int k = 10000;

	LinModel stateModel(2,2);
	stateModel = 1, 0,
				 0, 1;
	LinModel controlModel(2,2);
	controlModel =  1, 0,
					0, 1;
	LinModel measurementModel(2,2);
	measurementModel =  1, 0,
						0, 1;

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
		testFilter.update(u(i), z(i));
		StateEstimateArray(i).resize(2);
		StateEstimateArray(i) = testFilter.stateEstimate();
		CovarianceArray(i).resize(2,2);
		CovarianceArray(i) = testFilter.covariance();
	}
#endif
	//plot


	DenseVector<Array<double> > y(10);
	y = 1, 3, 5, 6, 7, 7, 4, 3, 2, 1;

	/*for(int i = 1; i < k; i++) {
		y(i).resize(2);
		y(i) = x(i+1) - StateEstimateArray(i);
		y1(i) = y(1)(1);
	}*/

	Gnuplot plotter;
	plotter.execute("plot 'test.dat' using linespoints\n" "pause mouse");
	//plotter.plot(y);


#if 0
	// is there a better way to do this?
	ofstream commandFile;
	commandFile.open("test.p");
	commandFile << "plot 'test.dat'\n" << "pause mouse" << endl;
	commandFile.close();

	ofstream dataFile;
	dataFile.open("test.dat");
	dataFile << "#test.txt\n";
	StateVector StateError(2);
	for(int i = 1; i < k; i++) {
		StateError = x(i+1) - StateEstimateArray(i);
		dataFile << i << "\t" << abs(StateError(1)) << "\n";
	}
	dataFile.close();

	system("gnuplot test.p");
#endif

	return(0);
}
