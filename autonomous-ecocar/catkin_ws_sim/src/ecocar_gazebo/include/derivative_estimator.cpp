#include "derivative_estimator.h"

DerivativeEstimator::DerivativeEstimator(float tc) {
	tau = tc;
	lastvalue = 0.0;
}

void DerivativeEstimator::update(float value, float dt) {
	float c1 = tau + 0.5*dt;
	float c2 = tau - 0.5*dt;

	y = (value - lastvalue) / c1 + c2 / c1 * y;

	lastvalue = value;
}


float DerivativeEstimator::value() {
	return y;
}
