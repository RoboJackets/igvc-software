DiscretePD::DiscretePD(int size) :
	bins(size) {
	
}

DiscretePD::DiscretePD(DenseVector<Array<Probability> prob) :
	bins(prob) {
	
}

DiscretePD::DiscretePD(E minVal, E maxVal, DenseVector<Array<Probability> prob) :
	bins(prob) {
	
}

DiscretePD::DiscretePD(DEVector ranges, inclusivity = all, DenseVector<Array<Probability> prob) :
	bins(size) {
	
}


Probability DiscretePD::operator() (E value) {
	// search for the right range
	for() {
	
	}
	return();
}


#if 0
DiscretePD::void convert(DEVector values, bool scaleValues = true) {

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
