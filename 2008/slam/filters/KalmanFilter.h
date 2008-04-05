#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

/* TODO:
 *	use SPMatrices
 *	make this use a pdf class
 *  make this inherit from a general filter class
 *  modifiy this to take multiple element types at a time (e.g. double and boolian values)
 */

//#include "BayesianFilter.h"
#include <flens/flens.h>
#include "NormalPD.h"
//#include "IdenMatrix.h"

using namespace flens;
using namespace std;

// these should be moved inside the class
typedef GeMatrix<FullStorage<double,ColMajor> > GEMatrix, CovMatrix, IdenMatrix; /*for testing*/
typedef SpMatrix<PackedStorage<double,ColMajor,Upper> > SPMatrix;
typedef DenseVector<Array<double> > DVector, control, measurement, StateVector;
typedef DenseVector<Array<int> > PermutationMatrix;

typedef GEMatrix(*LinModel)(double); //this isn't a very good way to do this

//template<typename E>
class KalmanFilter {
public:
	/* Constructors */
	KalmanFilter(
		StateVector muInital, CovMatrix SigmaInital,
		LinModel stateModel, LinModel controlModel, LinModel measurementModel,
		CovMatrix processNoise, CovMatrix measurementNoise);

	KalmanFilter(
		NormalPD<double> initalStateBelief,
		LinModel stateModel, LinModel controlModel, LinModel measurementModel,
		CovMatrix processNoise, CovMatrix measurementNoise);

	/**/
	// could split this into two function
	void update(control u, measurement z, double deltaT);

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
	LinModel A;		// State Model
	LinModel B;		// Control Modelp(2)
	LinModel C;		// Measurement Model

	// Noise -- these might be moved inside of the models
	CovMatrix R;	// Process Noise Covariance
	CovMatrix Q;	// Measurement Noise Covariance

	IdenMatrix I;	// identity matrix used for calculations - could this efficeintly be added inside update?

	int size;
};

#endif /* KALMAN_FILTER_H */

