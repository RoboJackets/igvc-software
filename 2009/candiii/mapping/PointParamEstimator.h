#ifndef _POINT_PARAM_ESTIMATOR_H_
#define _POINT_PARAM_ESTIMATOR_H_

#include "ParameterEstimator.h"
#include <cv.h>

#define _P_  printf("line: %d, file(%s)\n", __LINE__, __FILE__ );

#include <omp.h>


/**
 *
 */

class PointParamEstimator : public ParameterEstimator < CvPoint2D32f  , double >
{
public:
	PointParamEstimator(double delta);

	/**
	 * Compute the line defined by the given data points.
	 * @param data A vector containing two 2D points.
	 * @param This vector is cleared and then filled with the computed parameters.
	 *
	 *        If the vector contains less than two points then the resulting parameters
	 *        vector is empty (size = 0).
	 */
	virtual void estimate(std::vector< std::pair<CvPoint2D32f ,CvPoint2D32f >* > &data, std::vector<double> &parameters);

	/**
	 * Compute a least squares estimate of the line defined by the given points.
	 * This implementation is of an orthogonal least squares error.
	 *
	 * @param data The line should minimize the least squares error to these points.
	 * @param parameters This vector is cleared and then filled with the computed parameters.
	 *                   If the vector contains less than two points then the resulting parameters
	 *                   vector is empty (size = 0).
	 */
	virtual void leastSquaresEstimate(std::vector<std::pair<CvPoint2D32f ,CvPoint2D32f >* > &data, std::vector<double> &parameters);

	/**
	 * Return true if the distance between the line defined by the parameters and the
	 * given point is smaller than 'delta' (see constructor).
	 * @param parameters
	 * @param data Check that the distance is smaller than 'delta'.
	 */
	virtual bool agree(std::vector<double> &parameters, std::pair<CvPoint2D32f ,CvPoint2D32f >  &data);







private:
	double m_deltaSquared; //
	int computeHomography(CvPoint2D32f* p1, CvPoint2D32f* p2, CvMat* h);
};

#endif


