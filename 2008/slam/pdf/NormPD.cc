#include "NormPD.h"

NormPD(int size) : mu(size), Sigma(size) {
}

// Is this a shallow or deep copy?
NormPD( MeanVector mu, CovMatrix Sigma) : mu(mu), Sigma(Sigma) {
}

