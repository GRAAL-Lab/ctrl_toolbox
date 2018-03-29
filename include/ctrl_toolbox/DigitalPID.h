/*
 * DigitalPID.h
 *
 *  Created on: March 28, 2018
 *      Author: wanderfra
 */

#ifndef SRC_CTRL_DIGITALPID_H_
#define SRC_CTRL_DIGITALPID_H_

#include <cstdint>
#include <functional>

namespace ctb
{

struct PIDGains {
	double Kp = {0.0};  //!< Proportional gain
	double Ki = {0.0};  //!< Integral gain
	double Kd = {0.0};  //!< Derivative gain
	double Kff = {0.0}; //!< Feed-forward gain
	double N = {0.0};   //!< Maximum gain for derivative part
	double Tr = {0.0};  //!< Tracking time constant for anti-windup

	PIDGains() = default;
};

/**
 * @brief Difference functor
 *
 * An utility templated functor to compute the difference between two objects
 * of the same type.
 */
template <typename T>
struct DifferenceFunctor {
	T operator()(T a, T b) const{
		return (a - b);
	}
};

/**
 * \class DigitalPID
 *
 * \brief Impletation of a flexible 1-D digital PID controller
 *
 * This class implements a PID digital controller, with anti-windup and
 * feedforward control. The function to compute the error \f$ e \f$ can be configured
 * passing an std::function to SetErrorFunction(), function which has to take
 * two parameters \p reference and \p feedback and return the error.
 * The default error function is just the difference \f$ ref-fbk \f$, implemented
 * with the ctb::DifferenceFunctor. The output \f$ u \f$ is evaluated using the following
 * formula:\n
 *
 * \f$ u = K_p * e + I + D + K_{ff} * ref \f$
 *
 * where \f$ D \f$ is:\n
 *
 * \f$ D = \frac{T_d * D}{T_d + N * T_s} - \frac{K_p * T_d * N}{T_d + N * T_s} * (y_{diff}) \f$
 *
 */
class DigitalPID {
public:
	DigitalPID(const PIDGains& g, double sampleTime, double saturation);
	virtual ~DigitalPID();

	void SetGains(PIDGains g)	{
		Kp_ = g.Kp;
		Ki_ = g.Ki;
		Kd_ = g.Kd;
		Kff_ = g.Kff;
		N_ = g.N;
		Tr_ = g.Tr;
	}

	void SetSampleTime(double Ts) {
		Ts_ = Ts;
	}

	void SetSaturation(double uMax) {
		uMax_ = uMax;
	}

	void Reset();

	//double Compute(double error);
	double Compute(double ref, double fbk);

	void SetErrorFunction(const std::function<double(double, double)>& errorFunction)
	{
		ErrorFunction_ = errorFunction;
	}

private:
	double Kp_, Ki_, Kd_, Kff_;
	double Ts_;
	double uMax_;
	double u[2];
	double e[3];
	double y[2];
	double D_, I_, N_, Tr_;
	std::function<double(double, double)> ErrorFunction_;
	bool initialized_;
};

}

#endif /* SRC_CTRL_DIGITALPID_H_ */
