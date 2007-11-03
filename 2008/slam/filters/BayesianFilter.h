#ifndef BAYESIAN_FILTER_H
#define BAYESIAN_FILTER_H

/**
 * A stochastic filter which estimates the state of Hidden Marklov Models.
 * Transition Model
 * Measurment Model
 * @param S					the type of the state.
 * @param Z					the type of the measurements.
 */
template <typename S, typename Z>
class BayesianFilter
{
public:
	Pr<S> stateBel;
	virtual Pr<S> update(Z measurement);
	Pr<S> (*transModel)(Pr<S> previousStateBelief, w); // Transition Model
	Pr<Z> (*measureModel)(Pr<S> currentStateBelief, v); // Measurement Model
private:
		
}

#endif /* BAYESIAN_FILTER_H */

