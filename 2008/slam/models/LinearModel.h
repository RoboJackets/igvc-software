#ifndef LINEAR_MODEL_H
#define LINEAR_MODEL_H

/* TODO:
 *  make this inherit from a general model class
 *  modifiy this to take multiple element types at a time (e.g. double and boolian values)
 *  think about making noise a member of the model
 */

template<typename E>
class LinearModel {
public:
	/* Constructor */
	operator() (double deltaT);
	transpose();
private:
	GeMatrix<FullStorage<Equation<E>,ColMajor> > model;
}

#endif /* LINEAR_MODEL_H */

