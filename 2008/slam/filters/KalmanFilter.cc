#include "KalmanFilter.h"

//TODO: check that sizes are equal when they are set


/* Constructor */
KalmanFilter::KalmanFilter(
	StateVector muInital, CovMatrix SigmaInital,
	LinModel stateModel, LinModel controlModel, LinModel measurementModel,
	CovMatrix processNoise, CovMatrix measurementNoise)
:
	mu(muInital), Sigma(SigmaInital),
	A(stateModel), B(controlModel), C(measurementModel),
	R(processNoise), Q(measurementNoise),
	I(muInital.length(),muInital.length()) {
	//really crappy way to do an identity matrix
	size = muInital.length();
	for(int i = 1; i <= size; i++)
	{
		I(i,i) = 1;
	}
}

/**/
// FLENS blows
// TODO: explain the necessity of the temporary objects
void KalmanFilter::update(control u, measurement z) {

	GEMatrix tempM1(size, size);
	GEMatrix tempM2(size, size);
	/* Prediction */

	// mu = A * mu + B * u
	DVector tempV = A * mu;
	mu = tempV + B * u;

	// Sigma = A * Sigma * transpose(A) + R
	tempM1 = Sigma * transpose(A);
	Sigma = A * tempM1 + R;

	/* Measurement Update */

	// K = Sigma * transpose(C) * inv(C * Sigma * transpose(C) + Q)
	// This should be done using the inv() FLENS function, which hasn't been implemented yet.
	// This hacked version that uses (almost) direct LAPACK calls could be made faster by
	// using trs instead of tri.
	tempM1 = C * Sigma;
	tempM2 = tempM1 * transpose(C) + Q;
	PermutationMatrix p(size);
	int info = trf(tempM2,p);
	if(info) {
		printf("Error in KalmanFilter::update -- trs - info = %d\n", info);
		// what should be done here?
		//return;
	}
	info = tri(tempM2,p);
	if(info) {
		printf("Error in KalmanFilter::update -- tri - info = %d\n", info);
		//return;
	}
	GEMatrix K(size, size);
	K = Sigma * transpose(C);
	K = K * tempM2;

	//mu = mu + K * (z - C * mu);
	tempV = z - C * mu;
	mu = mu + K * tempV;

	//Sigma = (I - K * C) * Sigma;
	tempM1 = I - K * C;
	tempM2 = tempM1 * Sigma;
	Sigma = tempM2;

	return;
}

/* Accessors */
StateVector KalmanFilter::stateEstimate(void) {
	return(mu);
}

CovMatrix KalmanFilter::covariance(void) {
	return(Sigma);
}

//TODO: make a set() function that is overloaded to do all of these
int KalmanFilter::setModels(LinModel stateModel, LinModel controlModel, LinModel measurementModel) {
	A = stateModel;
	B = controlModel;
	C = measurementModel;
	return(0);
}

int KalmanFilter::setNoise(CovMatrix processNoise, CovMatrix measurementNoise) {
	R = processNoise;
	Q = measurementNoise;
}

