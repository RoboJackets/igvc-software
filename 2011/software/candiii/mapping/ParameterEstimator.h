#ifndef _PARAMETER_ESTIMATOR_H_
#define _PARAMETER_ESTIMATOR_H_

#include <vector>

/**
 * This class defines the interface for parameter estimators.
 * Classes which inherit from it can be used by the RanSac class to perfrom robust
 * parameter estimation.
 * The interface includes three methods:
 *                           1.estimate() - Estimation of the parameters using the minimal
 *                                          amount of data (exact estimate).
 *                           2.leastSquaresEstimate() - Estimation of the parameters using
 *                                                      more than the minimal amount of data, so that the estimate
 *                                                      minimizes a least squres error criteria.
 *                           3.agree() - Does the given data agree with the model parameters.
 *
 * Author: Ziv Yaniv (zivy@cs.huji.ac.il)
 */

template< class T, class S>
class ParameterEstimator
{
public:
	/**
	 * Exact estimation of parameters.
	 * @param data The data used for the estimate.
	 * @param parameters This vector is cleared and then filled with the computed parameters.
	 */
	virtual void estimate(std::vector< std::pair<T,T>* > &data, std::vector<S> &parameters) = 0;


	/**
	 * Least squares estimation of parameters.
	 * @param data The data used for the estimate.
	 * @param parameters This vector is cleared and then filled with the computed parameters.
	 */
	virtual void leastSquaresEstimate(std::vector< std::pair<T,T>* > &data, std::vector<S> &parameters) = 0;

	/**
	 * This method tests if the given data agrees with the given model parameters.
	 */
	virtual bool agree(std::vector<S> &parameters,  std::pair<T,T> &data) = 0;
};

#endif //_PARAMETER_ESTIMATOR_H_

