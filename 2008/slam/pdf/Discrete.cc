#include "Discrete.h"

DiscretePDF::DiscretePDF(int size) :
	bins(size) {
	
}

DiscretePDF::DiscretePDF(DenseVector<Array<Probability> prob) :
	bins(prob) {
	
}

DiscretePDF::DiscretePDF(E minVal, E maxVal, DenseVector<Array<Probability> prob) :
	bins(prob) {
	
}

DiscretePDF::DiscretePDF(DEVector ranges, inclusivity = all, DenseVector<Array<Probability> prob) :
	bins(size) {
	
}


Probability DiscretePDF::operator() (E value) {
	// search for the right range
	for() {
	
	}
	return();
}


#if 0
DiscretePDF::void convert(DEVector values, bool scaleValues = true) {

	for(int i = 0; i < values.length; i++) {
		++bins( floor(values(i) / stride) );
	}

	if(scaleValues) {
		for(int i = 0; i < bins.length; i++) {
			dist(i) /= (double)values.length;
		}
	}
}
#endif
