DiscretePD::DiscretePD(int size) :
	bins(size) {
	
}

DiscretePD::DiscretePD(DenseVector<Array<Probability> prob) :
	bins(prob) {
	
}

DiscretePD::DiscretePD(E minVal, E maxVal, DenseVector<Array<E> prob) :
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

