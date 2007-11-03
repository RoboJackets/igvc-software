#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

/* TODO:
 *	use SPMatrices
 *	write a debug macro that checks the value of info
 *	make this use a model class
 *  make this inherit from a general filter class
 *	make this use a Probablity distrubtion class
 *  modifiy this to take multiple element types at a time (e.g. double and boolian values)
 */

//#include "BayesianFilter.h"
#include <flens/flens.h>
//#include "GaussianPD.h"
//#include "IdenMatrix.h"

using namespace flens;
using namespace std;

// these should be moved somewhere else
typedef GeMatrix<FullStorage<double,ColMajor> > GEMatrix, LinModel, CovMatrix, IdenMatrix; /*for testing*/
typedef SpMatrix<PackedStorage<double,ColMajor,Upper> > SPMatrix;
typedef DenseVector<Array<double> > DVector, control, measurement, StateVector;
typedef DenseVector<Array<int> > PermutationMatrix;

//template<typename E>
class KalmanFilter {
public:
	/* Constructor */
	KalmanFilter(
		StateVector muInital, CovMatrix SigmaInital,
		LinModel stateModel, LinModel controlModel, LinModel measurementModel,
		CovMatrix processNoise, CovMatrix measureNoise);

	/**/
	// could split this into two function
	void update(control u, measurement z);

	/* Accessors */
	StateVector stateEstimate(void);
	CovMatrix covariance(void);

	int setModels(LinModel stateModel, LinModel controlModel, LinModel measurementModel);

	int setNoise(CovMatrix processNoise, CovMatrix measurementNoise);

private:

	/* Current state belief */
	StateVector mu;
	CovMatrix Sigma;

	// Models
	/*LinModel A;		// State Model
	LinModel B;		// Control Modelp(2)
	LinModel C;		// Measurement Model*/
	//for testing
	GEMatrix A, B, C;

	// Noise -- these might be moved inside of the models
	CovMatrix R;	// Process Noise Covariance
	CovMatrix Q;	// Measurement Noise Covariance

	IdenMatrix I;	// identity matrix used for calculations - could this efficeintly be added inside update?

	int size;
};

#endif /* KALMAN_FILTER_H */

