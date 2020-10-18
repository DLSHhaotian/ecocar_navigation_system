#ifndef DERIVATIVE_ESTIMATOR_H
#define DERIVATIVE_ESTIMATOR_H

/*  Derivative estimator with support for varying sampling time.
	Transfer function: s/(tau*s+1)
	Author: Thomas Passer Jensen (s134234)
*/

class DerivativeEstimator {
public:
	DerivativeEstimator(float tc);
	void update(float value, float dt);
	float value();

private:
	float tau;
	float lastvalue;
	float y;
};

#endif
