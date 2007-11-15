#include "KalmanFilter.h"
#include <flens/inv.h>

//TODO: check that sizes are equal when they are set


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

KalmanFilter::KalmanFilter(
	NormalPD<double> initalStateBelief,
	LinModel stateModel, LinModel controlModel, LinModel measurementModel,
	CovMatrix processNoise, CovMatrix measurementNoise)
:
	mu(initalStateBelief.mean()), Sigma(initalStateBelief.var()),
	A(stateModel), B(controlModel), C(measurementModel),
	R(processNoise), Q(measurementNoise),
	I(processNoise.numRows(),processNoise.numRows()) {
	//really crappy way to do an identity matrix
	size = processNoise.numRows();
	for(int i = 1; i <= size; i++)
	{
		I(i,i) = 1;
	}
}


// FLENS blows
// TODO: explain the necessity of the temporary objects
void KalmanFilter::update(control u, measurement z, double deltaT) {

	CovMatrix tempM1(size, size);
	CovMatrix tempM2(size, size);
	/* Prediction */

	// mu = A * mu + B * u
	DVector tempV = A(deltaT) * mu;
	mu = tempV + B(deltaT) * u;

	// Sigma = A * Sigma * transpose(A) + R
	tempM1 = Sigma * transpose(A(deltaT));
	Sigma = A(deltaT) * tempM1 + R;

	/* Measurement Update */

	// K = Sigma * transpose(C) * inv(C * Sigma * transpose(C) + Q)
	tempM1 = C(deltaT) * Sigma;
	tempM2 = tempM1 * transpose(C(deltaT)) + Q;
	tempM1 = inv(tempM2);
	GEMatrix K(size, size);
	K = Sigma * transpose(C(deltaT));
	K = K * tempM1;

	//mu = mu + K * (z - C * mu);
	tempV = z - C(deltaT) * mu;
	mu = mu + K * tempV;

	//Sigma = (I - K * C(deltaT)) * Sigma;
	tempM1 = I - K * C(deltaT);
	tempM2 = tempM1 * Sigma;
	Sigma = tempM2;

	return;
}


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

