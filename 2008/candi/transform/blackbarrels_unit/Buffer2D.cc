#include "Buffer2D.h"

namespace blackbarrels{

// ### ACCESSORS ###

template<class E>
E* Buffer2D<E>::atRow (int y) {
	return &data[y*width];
}

// ### OPERATIONS ###

// (see header file - I couldn't get the functions to
//  link when they were written here)
}
